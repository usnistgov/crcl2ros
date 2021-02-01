## <a name="Q:_What_is_the_difference_between_param_vs_rosparam?"></a>Q: What is the difference between param vs rosparam?


A: From wiki.ros.org/ROS/Patterns/Parameterization#Static_Parameters

<param> tags are for setting a single parameter and <rosparam> tags are for setting groups or more complex structures of parameters.


The value set by the <param> tag may only be a **string** , **int** , **bool** , or **double** , which may be set through the xml attribute value, or by reading in from a text file, bin file, or the output of a command line command.


The <rosparam> tag enables users to define a batch of related parameters simultaneously. These can be read from as YAML[<u>here</u>](https://yaml.org/) string which can either be put inline in the launchfile or can be loaded from a file (the rosparam dump command generates YAML output). Unlike the <param> tag, the YAML specification allows nested structures of parameters and parameters with list values. While these data can be represented on the ROS parameter server, it is up to the various client libraries to support complex parameter access.


The <rosparam> tag use in practice when there are many, many parameters to be set, it's less cumbersome to just use the YAML format.
