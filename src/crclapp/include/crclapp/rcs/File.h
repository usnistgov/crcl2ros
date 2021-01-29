
/*
 * DISCLAIMER:
 * This software was produced by the National Institute of Standards
 * and Technology (NIST), an agency of the U.S. government, and by statute is
 * not subject to copyright in the United States.  Recipients of this software
 * assume all responsibility associated with its operation, modification,
 * maintenance, and subsequent redistribution.
 *
 * See NIST Administration Manual 4.09.07 b and Appendix I.
 */


#ifndef _CFILE_H
#define  _CFILE_H


#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/regex.hpp>
#include <ctime>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <iterator>
#include <algorithm>
#include <string>

typedef bool boolean;


// References:
// https://www.technical-recipes.com/2014/using-boostfilesystem/#Iterating
// https://www.boost.org/doc/libs/1_60_0/libs/filesystem/doc/reference.html

class File {
    std::string module;
    std::string filename;
    std::string path;
    bool bDelete;
public:

    /**
   * @brief File  Creates a new File instance by converting the given pathname std::string into an abstract pathname.

   * @param pathname
   */
    File(std::string pathname)
    {
        this->path=pathname;
        bDelete=false;
    }
    ~File()
    {
        if(this->exists() && bDelete)
            remove(path.c_str());
    }

    /**
     * @brief getName Returns the name of the file or directory denoted by this abstract pathname.
     * @return std::string containing name
     */
    std::string 	getName()
    {
        return this->path;
    }

    /**
     * @brief deleteNow Deletes the file or directory denoted by this abstract pathname.
     * @return std::string containing name
     */

    boolean deleteNow()
    {
        if(this->exists())
        {
            remove(path.c_str());
            return true;
        }
        return false;
    }

    /**
     * @brief deleteOnExit Requests that the file or directory denoted by this abstract pathname be deleted when the virtual machine terminates.
     */
    void 	deleteOnExit()
    {
        bDelete=true;
    }



    /**
   * @brief canExecute Tests whether the application can execute the file denoted by this abstract pathname.
   * From https://stackoverflow.com/questions/8812959/how-to-read-linux-file-permission-programmatically-in-c-c
   * @return true if file can be executed by user.
   */
    bool canExecute()
    {
        struct stat st;
        if(stat(path.c_str(), &st) == 0){
            mode_t perm = st.st_mode;
            if (perm & S_IXUSR)
                return true;
        }
        return false;
    }

    /**
   * @brief canRead Tests whether the application can read the file denoted by this abstract pathname.
   * @return
   */
    bool 	canRead()
    {
        std::ifstream my_file(path);
        if (my_file.good())
        {
            return true;
        }
        return false;
    }

    /**
   * @brief canWrite Tests whether the application can modify the file denoted by this abstract pathname.
   * @return
   */
    bool 	canWrite()
    {
        bool ret=true;
        FILE *fp = fopen(path.c_str(), "w");
        if (fp == NULL) {
            if (errno == EACCES)
                ret=false;
            else
                ret=false;
        }
        fclose(fp);
        return ret;
    }

    std::string getCanonicalPath()
    {
        return path;
    }

    std::string getParentFile()
    {
        std::string mypath(path);
        return mypath.substr(0, mypath.find_last_of(PathSeparator()) + 1);

    }

    /**
   * @brief createNewFile Atomically creates a new, empty file named by this abstract pathname if and only if a file with this name does not yet exist.
   * @return  true if permissions allow you to create the file
   */
    boolean 	createNewFile()
    {
        if(!this->exists())
        {
            assert(0);
            return true;
        }
        assert(0);
        return false;
    }

    /**
   * @brief createTempFile Creates an empty file in the default temporary-file directory, using the given prefix and suffix to generate its name.
   * @param prefix prefix of name to use before .
   * @param suffix extension of name after .
   * @return
   */
      static File createTempFile(std::string prefix, std::string suffix)
      {
          std::string filename= boost::filesystem::temp_directory_path().c_str();
          filename+= "/"+ prefix+"." +suffix ; /// for a complete temporary pathname.
          File f(filename);
          f.createNewFile();
          return f;
      }

      /**
       * @brief createTempFile Creates a new empty file in the specified directory, using the given prefix and suffix std::strings to generate its name.

       * @param prefix prefix of name to use before .
       * @param suffix extension of name after .
       * @param directory
       * @return
       */
      static File 	createTempFile(std::string prefix, std::string suffix, File directory)
      {
          std::string filename= directory.getName();
          filename+= "/" + prefix + "." +suffix ; /// for a complete temporary pathname.
          File f(filename);
          f.createNewFile();
          return f;

      }

      /**
       * @brief compareTo Compares two abstract pathnames lexicographically.
       * https://stackoverflow.com/questions/6163611/compare-two-files
       * @param pathname
       * @return false unequal, true equal
       */
      int compareTo(File pathname)
      {
          std::string p1(this->getName());
          std::string p2(pathname.getName());
          std::ifstream f1(p1, std::ifstream::binary|std::ifstream::ate);
          std::ifstream f2(p2, std::ifstream::binary|std::ifstream::ate);

          if (f1.fail() || f2.fail()) {
              return false; //file problem
          }

          if (f1.tellg() != f2.tellg()) {
              return false; //size mismatch
          }

          //seek back to beginning and use std::equal to compare contents
          f1.seekg(0, std::ifstream::beg);
          f2.seekg(0, std::ifstream::beg);
          return std::equal(std::istreambuf_iterator<char>(f1.rdbuf()),
                            std::istreambuf_iterator<char>(),
                            std::istreambuf_iterator<char>(f2.rdbuf()));
      }
    /**
     * @brief renameTo Renames the file denoted by this abstract pathname.
     * @param dest file with path to rename file to.
     * @return true if renamed
     */
    boolean 	renameTo(File dest)
    {
        std::string filename =dest.getName();
        int result= rename(this->getName().c_str(), filename.c_str());
        return result == 0;

    }


    /**
   * @brief Exists tests whether the file or directory denoted by this abstract pathname exists.
   * @param filename pathname of file
   * @return true if exiss
   */
    bool exists() {
        return boost::filesystem::exists(path);
    }

    /**
   * @brief length  Returns the length of the file denoted by this abstract pathname.
   * @return
   */
    long 	length()
    {
        return boost::filesystem::file_size(path);
    }

    /**
   * @brief lastModified Returns the time that the file denoted by this abstract pathname was last modified.
   * @param filename
   * @return
   */
    std::time_t lastModified() {
        boost::filesystem::path p(path);
        return boost::filesystem::last_write_time(p);
    }


    /**
     * @brief isAbsolute  Tests whether this abstract pathname is absolute.
     * @return
     */
    boolean 	isAbsolute()
    {
        struct stat stbuf;
        stat(path.c_str(),&stbuf) ;
        switch(stbuf.st_mode & S_IFMT){
        case S_IFLNK:
            return false;
        case S_IFREG:
            return true;
        }
    }


    /**
     * @brief isDirectory Tests whether the file denoted by this abstract pathname is a directory.
     * @return
     */
    boolean 	isDirectory()
    {
        struct stat s;
        if( stat(path.c_str(),&s) == 0 )
        {
            if( s.st_mode & S_IFDIR )
            {
                return true;
            }
            else if( s.st_mode & S_IFREG )
            {
                return false;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    /**
     * @brief isFile Tests whether the file denoted by this abstract pathname is a normal file.
     * @return
     */
    boolean 	isFile()
    {
        struct stat s;
        if( stat(path.c_str(),&s) == 0 )
        {
            if( s.st_mode & S_IFDIR )
            {
                return false;
            }
            else if( s.st_mode & S_IFREG )
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    /**
     * @brief isHidden Tests whether the file named by this abstract pathname is a hidden file.
     * @return
     */
    boolean 	isHidden()
    {
        return false;
    }



    int Size(std::string filename, long long &size) {
        size = boost::filesystem::file_size(filename);
        return 0;
    }

    std::string GetFileModTimeStr(std::string filename) {
        std::stringstream s;
        boost::filesystem::path p(filename);
        std::time_t ft = last_write_time(p);

        s << std::asctime(std::gmtime(&ft)) << std::endl;
        return s.str();
    }
    static std::string PathSeparator()
    {
#ifdef WIN32
        return "\\";
#else
        return "/";
#endif
        // segmentation fault?
        //    return std::string(boost::filesystem::path::preferred_separator);
    }



    static int do_mkdir(const char *path, mode_t mode)
    {
        struct stat     st;
        int             status = 0;

        if (stat(path, &st) != 0)
        {
            /* Directory does not exist. EEXIST for race condition */
            if (::mkdir(path, mode) != 0 && errno != EEXIST)
                status = -1;
        }
        else if (!S_ISDIR(st.st_mode))
        {
            errno = ENOTDIR;
            status = -1;
        }

        return(status);
    }

    /**
    * mkpath - ensure all directories in path exist
    * Algorithm takes the pessimistic view and works top-down to ensure
    * each directory in path exists, rather than optimistically creating
    * the last element and working backwards.
    */
    int mkpath(const char *path, mode_t mode)
    {
        char           *pp;
        char           *sp;
        int             status;
        char           *copypath = strdup(path);

        status = 0;
        pp = copypath;
        while (status == 0 && (sp = strchr(pp, '/')) != 0)
        {
            if (sp != pp)
            {
                /* Neither root nor double slash in path */
                *sp = '\0';
                status = do_mkdir(copypath, mode);
                *sp = '/';
            }
            pp = sp + 1;
        }
        if (status == 0)
            status = do_mkdir(path, mode);
        free(copypath);
        return (status);
    }


    /**
   * @brief mkdirs Creates the directory named by this abstract pathname,
   *  including any necessary but nonexistent parent directories.
   * @return true if all folders were created.
   */
    boolean 	mkdirs()
    {
        return mkpath(path.c_str(), 0777);
    }



    std::string TempDirectory() { return "/tmp"; }
    std::string ExeDirectory() {
        assert(!module.empty());
        std::string path(module);
        path = path.substr(
                    0, path.find_last_of(boost::filesystem::path::preferred_separator) + 1);
        return path;
    }
    std::string ExtractDirectory(const std::string &path) {
        return path.substr(0, path.find_last_of(PathSeparator()) + 1);
    }
    std::string ExtractFilename(const std::string &path) {
        return path.substr(path.find_last_of(PathSeparator()) + 1);
    }
    std::string ExtractFiletitle(const std::string &path) {
        std::string filename = path.substr(path.find_last_of(PathSeparator()) + 1);

        return filename.substr(0, filename.find_last_of('.'));
    }
    std::string ChangeExtension(const std::string &path,
                                const std::string &ext) {
        std::string filename = ExtractFilename(path);

        return ExtractDirectory(path) +
                filename.substr(0, filename.find_last_of('.')) + ext;
    }
    std::string GetModuleName() {
        assert(!module.empty());
        return ExtractFilename(std::string(module));
    }
    std::string Filename(std::string path) {
        size_t sep =
                path.find_last_of(boost::filesystem::path::preferred_separator);

        if (sep != std::string::npos) {
            return path.substr(sep + 1);
        }

        return path; // throw?
    }
    std::string Filetitle(std::string path) {
        path = Filename(path);
        size_t dot = path.find_last_of(".");

        if (dot != std::string::npos) {
            return path.substr(0, dot);
        }

        return path;
    }
    std::string Extension(std::string path) {
        size_t dot = path.find_last_of(".");

        if (dot != std::string::npos) {
            return path.substr(dot);
        }

        return ""; // throw?
    }

    /**
     * @brief list returns an array of std::strings naming the files and directories
     * in the directory denoted by this abstract pathname.
     * OR for(auto & p : boost::filesystem::directory_iterator( path )){ std::cout << p << std::endl; }
     * OR https://felixmorgner.com/blog/c++/2017/06/17/filtering-files-with-boost.html
     * @return vector of contents
     */
    std::vector<std::string> 	list()
    {
        std::vector<std::string> m_file_list;
        if (!path.empty())
        {
            namespace fs = boost::filesystem;

            fs::path apk_path(path);
            //fs::recursive_directory_iterator end;
            //   for (fs::recursive_directory_iterator i(apk_path); i != end; ++i)
            fs::directory_iterator end;
            for (fs::directory_iterator i(apk_path); i != end; ++i)
            {
                const fs::path cp = (*i);
                m_file_list.push_back(cp.string());
            }
        }
        return m_file_list;
    }

    /**
     * @brief list Returns an array of std::strings naming the files and directories in the directory
     * denoted by this abstract pathname that satisfy the specified filter.
     * see https://theboostcpplibraries.com/boost.regex also
     * TODO: change from boost to std c11
     * @param filter
     * @return vector of path names
     */
    std::vector<std::string>	list(std::string filefilter)
    {
        std::vector<std::string> candidates=list();
        boost::regex expr{filefilter};
        for(size_t i=0; i< candidates.size(); i++)
        {
            std::string s(candidates[i]);
            if(! boost::regex_match(s, expr))
            {
                candidates.erase (candidates.begin()+i);
                i--;
            }
        }
        return candidates;
#if 0
        std::vector<std::string> results;
        boost::filesystem::path filepath(this->path);
        boost::filesystem::directory_iterator it(filepath);
        boost::filesystem::directory_iterator end;
        const boost::regex pidFileFilter(filefilter);
        BOOST_FOREACH(boost::filesystem::path const &p, std::make_pair(it, end))
        {
            if(boost::filesystem::is_regular_file(p))
            {
                boost::match_results<std::string::const_iterator> what;
                if (regex_search(it->path().filename().std::string(), what, pidFileFilter, boost::match_default))
                {
                    std::string res = what[1];
                    results.push_back(res);
                }
            }
        }
        return results;
#endif
    }
    /**
     * @brief setExecutable A convenience method to set the owner's execute permission
     * for this abstract pathname.
     * https://stackoverflow.com/questions/29068909/removing-permissions-on-a-file-c-programming
     * @param executable boolean
     * @return true if executable permission was set.
     */
    boolean setPermission(boolean bAdded, mode_t permission=S_IXUSR)
    {
        if(!exists())
            return false;

        struct stat st;
        mode_t mode;
        stat(path.c_str(), &st);

        mode = st.st_mode & 07777;

        // modify mode
       if(bAdded)
           mode |= permission;       /* Set this bit   */
       else
           mode &= ~(permission);    /* Clear this bit */

        if( chmod(path.c_str(), mode))
            return false;
        return true;
    }
    /**
     * @brief setExecutable A convenience method to set the owner's execute permission
     * for this abstract pathname.
     * https://stackoverflow.com/questions/29068909/removing-permissions-on-a-file-c-programming
     * @param executable boolean
     * @return true if executable permission was set.
     */
    boolean setExecutable(boolean executable)
    {
        return setPermission(executable, S_IXUSR);
    }

    /**
     * @brief setExecutable Sets the owner's or everybody's execute permission for this abstract pathname.
     * @param executable
     * @param ownerOnly
     * @return
     */
    boolean 	setExecutable(boolean executable, boolean ownerOnly)
    {
        mode_t permission = S_IXUSR;
        if(!ownerOnly)
            permission|=S_IXGRP;
        return setPermission(executable, permission);
    }

    /**
     * @brief setReadable A convenience method to set the owner's read permission for this abstract pathname.
     * @param readable
     * @return
     */
    boolean setReadable(boolean readable)
    {
        return setPermission(readable, S_IRUSR);
    }

    /**
     * @brief setReadable Sets the owner's or everybody's read permission for this abstract pathname.
     * @param readable
     * @param ownerOnly
     * @return
     */
    boolean 	setReadable(boolean readable, boolean ownerOnly)
    {
        mode_t permission = S_IRUSR;
        if(!ownerOnly)
            permission|=S_IRGRP;
        return setPermission(readable, permission);

    }

    /**
     * @brief setReadOnly Marks the file or directory named by this abstract pathname
     * so that only read operations are allowed.
     * @return
     */
    boolean 	setReadOnly()
    {
        // remove all permission first
        struct stat st;
        mode_t mode;
        stat(path.c_str(), &st);
        mode = st.st_mode & 07777;
        setPermission(0, mode);
        return setPermission(1, S_IRUSR|S_IRGRP|S_IROTH);

    }

    /**
     * @brief setWritable A convenience method to set the owner's
     * write permission for this abstract pathname.
     * @param writable
     * @return
     */
    boolean 	setWritable(boolean writable)
    {
        return setPermission(writable, S_IWUSR);

    }

    /**
     * @brief setWritable Sets the owner's or everybody's write permission for this abstract pathname.
     * @param writable
     * @param ownerOnly
     * @return
     */
    boolean 	setWritable(boolean writable, boolean ownerOnly)
    {
        mode_t permission = S_IWUSR;
        if(!ownerOnly)
            permission|=S_IWGRP;
        return setPermission(writable, permission);
    }

    /**
     * @brief tostd::string Returns the pathname std::string of this abstract pathname.
     * @return
     */
    std::string 	toString()
    {
        return path;
    }

//    /**
//     * @brief toURI Constructs a file: URI that represents this abstract pathname.
//     * @return
//     */
//    URI 	toURI()
//    {
//        assert(0);
//    }

    /**
     * @brief mkdir Creates the directory named by this abstract pathname.
     * @return
     */
    boolean 	mkdir()
    {
        return boost::filesystem::create_directory(boost::filesystem::path(this->path));
    }

    /**
     * @brief setLastModified  Sets the last-modified time of the file or directory named by this abstract pathname.
     * @param time
     * @return
     */
    boolean setLastModified(time_t new_time)
    {
        namespace fs = boost::filesystem;
        if(!exists())
            return false;
        fs::last_write_time(fs::path(this->path),  new_time);
        return true;
    }


    /**
     * @brief getFreeSpace  Returns the number of unallocated bytes
     * in the partition named by this abstract path name.
     * @return free in bytes
     */
    long 	getFreeSpace()
    {
        namespace fs = boost::filesystem;
        fs::space_info si = fs::space(fs::path(this->path));
        return si.free;
    }

    /**
     * @brief getTotalSpace  Returns the size of the partition named by this abstract pathname.
     * @return  size in bytes
     */
    long 	getTotalSpace()
    {
        namespace fs = boost::filesystem;
        fs::space_info si = fs::space(fs::path(this->path));
        return si.capacity;
    }

    /**
     * @brief getUsableSpace  Returns the number of bytes available
     * to this virtual machine on the partition named by this abstract pathname.
     * @return
     */
    long 	getUsableSpace()
    {
        namespace fs = boost::filesystem;
        fs::space_info si = fs::space(fs::path(this->path));
        return si.available;
    }


};
#endif
