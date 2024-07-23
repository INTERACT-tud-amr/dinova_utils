# dinova_utils
A customized diova utils package to coorperate with dinova_mpc package.

        self.object_names = rospy.get_param("vicon/object_specific/object_names")
        self.objects_to_remove = rospy.get_param("objects_to_remove")
        self.radius_objects = rospy.get_param("radius_objects")



## 1. dinova robot ros interface
This is an interface used together with ocs2 mrt node to receive newset dinova robot states and publish commands by ros topics. For flexibility, all topics are not hardcoded and they need to be remapped in launch file of the mrt node.   
#### Compulsary Parameters

* **`~vicon/object_specific/object_names`** (list of string)

* **`~objects_to_remove`** (list of string)

* **`~radius_objects`** (list of float)

#### Subscribed Topics

* **`/vicon/<object_name>`** ([geometry_msg/PoseStamped])



#### Published Topics

* **`~objects`** ([derived_object_msg/ObjectArray])


<!-- kinova_cmd_topic -->