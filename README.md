# dinova_utils
A customized diova utils package to coorperate with dinova_mpc package.

#### Compulsary Parameters

* **`~vicon/object_specific/object_names`** (list of string)

* **`~objects_to_remove`** (list of string)

* **`~radius_objects`** (list of float)

#### Subscribed Topics

* **`/vicon/<object_name>`** ([geometry_msg/PoseStamped])



#### Published Topics

* **`~objects`** ([derived_object_msg/ObjectArray])


<!-- kinova_cmd_topic -->