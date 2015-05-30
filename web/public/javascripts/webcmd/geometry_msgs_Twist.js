
function geometry_msgs_Twist_onInit(element, box, boxReference) {

	var x = 0.0
	var z = 0.0
	
	//Set defaults
	if (box.params.wasd === undefined) {
		box.params.wasd = [87, 65, 83, 68]
	}
	if (box.params.pub_topic === undefined)
		box.params.pub_topic = "cmd_vel";

	if (box.params.max_forward === undefined)
		box.params.max_forward = 0.4;

	if (box.params.max_twist === undefined)
		box.params.max_twist = 2.4;

	if (box.params.accel === undefined)
		box.params.accel = 1;

	//TODO calculate acceleration based on time from start, separate forward and twist
	step_x = box.params.max_forward / (box.params.accel * 10);
	step_z = step_x * 3;

	//register w key
	hotkeys.addKey(new HotKey(box.params.wasd[0], function() {
		x += step_x;
		if (x > box.params.max_forward)
			x = box.params.max_forward;
		publish_cmd_vel(x,z, topic);
	},
	function() {
		x = 0;
		publish_cmd_vel(x,z, topic);
	}));

	//register s key
	hotkeys.addKey(new HotKey(box.params.wasd[2], function() {
		x -= step_x;
		if (x < -box.params.max_forward)
			x = -box.params.max_forward;
		publish_cmd_vel(x,z, topic);
	},
	function() {
		x = 0;
		publish_cmd_vel(x,z, topic);
	}));
	
	//register a key
	hotkeys.addKey(new HotKey(box.params.wasd[1], function() {
		z += step_z;
		if (z > box.params.max_twist)
			z = box.params.max_twist;
		publish_cmd_vel(x,z, topic);
	},
	function() {
		z = 0;
		publish_cmd_vel(x,z, topic);
	}));
	
	
	//register d key
	hotkeys.addKey(new HotKey(box.params.wasd[3], function() {
		z -= step_z;
		if (z < -box.params.max_twist)
			z = -box.params.max_twist;
		publish_cmd_vel(x,z, topic);
	},
	function() {
		z = 0;
		publish_cmd_vel(x,z, topic);
	}));

	var topic = new ROSLIB.Topic({
		ros : ros,
		name : box.params.pub_topic,
		messageType : "geometry_msgs/Twist"
	});
	
	var html = "";
	html += "<style>.wasd_key { border: 3px #444 solid; height: 23px; float: left; width: 23px; margin: 2px; font-size: 13pt; text-align: center; vertical-align: middle; } </style>";
	html += "<div id='" + box.id + "_w' class='wasd_key' style='margin-left: 35px; '>" + box.params.wasd[0] + "</div>";
	html += "<div id='" + box.id + "_a' class='wasd_key' style='clear: both; '>" + box.params.wasd[1] + "</div>";
	html += "<div id='" + box.id + "_s' class='wasd_key'>" + box.params.wasd[2] + "</div>";
	html += "<div id='" + box.id + "_d' class='wasd_key'>" + box.params.wasd[3] + "</div>";
	element.html(html);
}

function publish_cmd_vel(x,z, topic) {
	var request = new ROSLIB.Message({
		linear: {
			x: x
		},
		angular: {
			z: z
		}
	});
	topic.publish(request);
}


