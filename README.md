# dinova_utils
A customized diova utils package to coorperate with dinova_mpc package.

#### Compulsary Parameters

* **`~vicon/object_specific/object_names`** (list of string)

    A list of strings contains all the object names whose Vicon topics the dinova_utils node will listen to.

* **`~objects_to_remove`** (list of string)

    A list of strings contains all the object names that `dinova_utils` will not insert into the published `~objects` topic.

* **`~radius_objects`** (list of float)

    Radius of each object

#### Subscribed Topics

* **`/vicon/<object_name>`** ([geometry_msg/PoseStamped])

    The topic published by vicon systems, providing pose information of the  `<object_name>` object.



#### Published Topics

* **`~objects`** ([derived_object_msg/ObjectArray])

    The topic stacks the information of all the objects and published them at once.


<!-- kinova_cmd_topic -->