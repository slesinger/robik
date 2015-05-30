var boxesconfig = {
	ros_host: "robik",
	ros_port: "9090",
	boxes: [

		{
			"id": "kinect",
			"label": "Kinect",  //optional - defaults to "id"
			"width": 6, //defaults to 1
			"height": 5, //defaults to 1
			"color": '#88FFCC', //defaults to random color
			"widget_type": "sensor_msgs/Image",
			"params": {
				"width" : 640,
				"height" : 480,
				"sub_topic" : ['/camera/rgb/image_raw', '/camera/depth/image_raw', '/cam_oko/image_raw']
			}
		},


		{
			id: "urdf3d",
			label: "Model",
			color: '#FFA020',
			widget_type: "urdf3d",
			width: 4,
			height:4,
			params: {
				map_topic: "/map",
				move_base: "/move_base",
			}
		},

		{
			"id": "arm_control",
			"label": "Ruka",
			"width": 2,
			"height": 2,
			"color": '#CCFFCC',
			"widget_type": "trajectory_msgs/JointTrajectory",
			"params": {
				"pub_topic": "robik_arm_controller_joint_states", //trajectory_msgs::JointTrajectory
				"sub_topic": "joint_states", //sensor_msgs::JointState, this is correct for moveit
				"joints": [
					{
						"id": "yaw_joint",
						"min": -1.8675023,
						"max": 1.099557429,
						"step": 0.2,
						"keys": ['B', 'N']
					},
					{
						"id": "shoulder_joint",
						"min": 0.0,
						"max": 2.094395102,
						"step": 0.1,
						"keys": ['G', 'T']
					},
					{
						"id": "elbow_joint",
						"min": -1.919862177,
						"max": 0.6981317008,
						"step": 0.1,
						"keys": ['H', 'Y']
					},
					{
						"id": "roll_joint",
						"min": -1.571,
						"max": 1.571,
						"step": 0.3,
						"keys": ['M', '¼'] //M ,
					},
					{
						"id": "clamp_joint",
						"min": 0.0,
						"max": 0.6981317008,
						"step": 0.1,
						"keys": ['J', 'K']
					},
					]
			}
		},

		{
			"id": "oko_control",
			"label": "Kamera oko",
			disabled: true,
			"width": 2,
			"height": 2,
			"color": '#CCCCCC',
			"widget_type": "sensor_msgs/JointState",
			"params": {
				"pub_topic": "robik_oko_controller_joint_states", //sensor_msgs::JointState
				"sub_topic": undefined, //sensor_msgs::JointState
				"joints": [
					{
						"id": "oko_yaw_joint",
						"min": 0.0,
						"max": 3.14,
						"step": 0.1,
						"keys": ['C', 'V']
					},
					{
						"id": "oko_pitch_joint",
						"min": 0.0,
						"max": 3.14,
						"step": 0.1,
						"keys": ['F', 'R']
					},
					]
			}
		},

		{
			"id": "arm_visual",
			"label": "Ruka",  //optional - defaults to "id"
			"width": 3, //defaults to 1
			"height": 2, //defaults to 1
			"color": '#CCFFCC', //defaults to random color
			"widget_type": "std_msgs/String",
			"params": {
				"pub_topic": undefined,   //conditional to sub_topic
				"sub_topic": "<img src='/images/ruka.png' style='width: 281px'/>" //conditional to pub_topic, can be also html string
			}
		},

		{
			"id": "arm_actions",
			"label": "Ruka - akce",
			"width": 2,
			"height": 2,
			"color": '#CCFFCC',
			"widget_type": "action_msgs/Button",
			"params": {
				"pub_topic": "/robik_action_arm/goal",
				"sub_topic": undefined, //mel by prijimat odezvu z akce
				"action_message_type": "robik/armActionGoal",
				"buttons": [
				            {
				            	"label": "Zaparkuj",
				            	"goal": {
				            		"command" : 1,
				            		"opTimeMs" : 2000
				            	}
				            },
				            {
				            	"label": "Priprav",
				            	"goal": {
				            		"command" : 2,
				            		"opTimeMs" : 2000
				            	}
				            },
				            {
				            	"label": "<",
				            	"goal": {
				            		"command" : 3,
				            		"opTimeMs" : 2000
				            	}
				            },
				            {
				            	"label": ">",
				            	"goal": {
				            		"command" : 4,
				            		"opTimeMs" : 2000
				            	}
				            },
				            {
				            	"label": "Chnapni",
				            	"goal": {
				            		"command" : 5,
				            		"opTimeMs" : 2000
				            	}
				            },
				            {
				            	"label": "Ukaz",
				            	"goal": {
				            		"command" : 6,
				            		"opTimeMs" : 2000
				            	}
				            },
				            {
				            	"label": "Zamavej",
				            	"goal": {
				            		"command" : 7,
				            		"opTimeMs" : 2000
				            	}
				            },
				            {
				            	"label": "Test 1",
				            	"goal": {
				            		"command" : 8,
				            		"opTimeMs" : 2000
				            	}
				            },
				            {
				            	"label": "Test 2",
				            	"goal": {
				            		"command" : 9,
				            		"opTimeMs" : 2000
				            	}
				            }
				]
			}
		},

		{
			"id": "move_actions",
			"label": "Pohyb - akce",
			"width": 2,
			"height": 2,
			"color": '#CCFFFF',
			"widget_type": "action_msgs/Button",
			"params": {
				"pub_topic": "/robik_action_move/goal",
				"sub_topic": "/robik_action_move/feedback",
				"action_message_type": "robik/moveActionGoal",
				"buttons": [
										{
											"label": "Zastav",
											"key": 'X',
											"goal": {
												"command" : 1
											}
										},
										{
											"label": "Narozeniny",
											"goal": {
												"command" : 13
											}
										},
										{
											"label": "Zaparkuj",
											"goal": {
												"command" : 2
											}
										}
				]
			}
		},

		{
			"id": "move_control",
			"label": "Pohyb",  //optional - defaults to "id"
			"width": 1, //defaults to 1
			"height": 1, //defaults to 1
			"color": '#CCCCFF', //defaults to random color
			"widget_type": "geometry_msgs/Twist",
			"params": {
				"pub_topic": "cmd_vel",   //defaults to cmd_vel
				"wasd": ['W', 'A', 'S', 'D'], //defaults to wasd
				"max_forward": 0.4, //defaults to 0.4
				"max_twist": 1.0,  //defaults to 2.4
				"accel": 3 //time [sec] in which max_* will be reached, defaults to 1
			}
		},

		{
			"id": "arm_fore_clamp",
			"label": "Ruka stisk predni",  //optional - defaults to "id"
			"color": '#CCFFCC',
			"widget_type": "std_msgs/Boolean",
			"params": {
				"sub_topic": "robik_status",
				"sub_message": "robik/GenericStatus",
				"sub_mapping": "arm_fore_clamp"
			}
		},

		{
			"id": "arm_back_clamp",
			"label": "Ruka stisk zadni",  //optional - defaults to "id"
			"color": '#CCFFCC',
			"widget_type": "std_msgs/Boolean",
			"params": {
				"sub_topic": "robik_status",
				"sub_message": "robik/GenericStatus",
				"sub_mapping": "arm_back_clamp"
			}
		},

		{
			"id": "sound_request",
			"label": "Rekni",  //optional - defaults to "id"
			"width": 2, //defaults to 1
			"height": 1, //defaults to 1
			"color": '#44CCFF', //defaults to random color
			"widget_type": "std_msgs/String",
			"params": {
				"pub_topic": "robik_ai",
				"key": '2',
				"sub_topic": "",
				"prepend": "rekni ",
				"prompt_label": "Zadej co ma rict"
			}
		},

		{
			"id": "ai",
			"label": "AI",  //optional - defaults to "id"
			"width": 2, //defaults to 1
			"height": 1, //defaults to 1
			"color": '#44CC44', //defaults to random color
			"widget_type": "std_msgs/String",
			"params": {
				"pub_topic": "robik_ai",
				"key": '1',
				"sub_topic": ""
			}
		},

		{
			"id": "light",
			"label": "Svetlo",
			"color": '#44FFFF',
			"widget_type": "std_msgs/Boolean",
			"params": {
				"pub_topic": "robik_generic_control",
				"pub_message": "robik/GenericControl",
				"pub_data": {
						gen_operation: 3,
						gen_param1: 0
					},
				"pub_mapping": function(data, val) {
					data.gen_param1 = (val == true) ? 1 : 0;
				}
			}
		},

		{
			"id": "bumber",
			"label": "Naraznik",
			"color": '#4400FF',
			"widget_type": "std_msgs/Boolean",
			"params": {
				"sub_topic": "robik_status",
				"sub_message": "robik/GenericStatus",
				"sub_mapping": "bumper_front"
			}
		},

		{
			"id": "movedetector",
			"label": "Pohyb",
			"color": '#4400FF',
			"widget_type": "std_msgs/Boolean",
			"params": {
				"sub_topic": "robik_status",
				"sub_message": "robik/GenericStatus",
				"sub_mapping": "motion_detector"
			}
		},

		{
			"id": "armpower",
			"label": "Napajeni ruky",
			"color": '#4444FF',
			"widget_type": "std_msgs/Boolean",
			"params": {
				"pub_topic": "robik_generic_control",
				"pub_message": "robik/GenericControl",
				"initial_value": true,
				"pub_data": {
						gen_operation: 5,
						gen_param1: 1
					},
				"pub_mapping": function(data, val) {
					data.gen_param1 = (val == true) ? 1 : 0;
				}
			}
		},

		{
			"id": "nav2d",
			"label": "Navigace",
			"color": '#22A0FF',
			"disabled": true,
			"widget_type": "nav2d",
			"width": 4,
			"height":4,
			"params": {
				"map_topic": "/map",
				"move_base": "/move_base",
			}
		}

	]
};

$(document).ready( function() {
	webcmd_init();
});
