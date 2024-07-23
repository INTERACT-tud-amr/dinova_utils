# dinova_utils
A customized diova utils package to coorperate with dinova_mpc package. This forked version separates static objects from object list and publish it to a independant topic.

#### Compulsary Parameters

* **`~vicon/object_specific/object_names`** (list of string)

    A list of strings contains all the object names whose Vicon topics the dinova_utils node will listen to. For real world applications, it is usaully set by the bringup launch file of dinova robot. For simulation, it need to be set manually.

* **`~objects_to_remove`** (list of string)

    A list of strings contains all the object names that `dinova_utils` will not insert into the published objects topic.

* **`~radius_objects`** (list of float)

    Radius of remaining objects

* **`~static_objects`** (list of float)

    A list of strings contains all the static object names. Curently hard-coded as [tablesmall_1, tablesmall_2]
    

#### Subscribed Topics

* **`/vicon/<object_name>`** ([geometry_msg/PoseStamped])

    The topic published by vicon systems, providing pose information of the  `<object_name>` object specified in `~vicon/object_specific/object_names` parameter.



#### Published Topics
* **`~static_objects`** ([derived_object_msg/ObjectArray])

    The topic stacks the information of all the static objects and published them at once.

* **`~objects`** ([derived_object_msg/ObjectArray])

    The topic stacks the information of all the other objects and published them at once.


<!-- kinova_cmd_topic -->