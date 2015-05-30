var trajectory_msgs_JointTrajectory_initialized = false;

function trajectory_msgs_JointTrajectory_onInit(element, box, boxReference) {

	var topic = new ROSLIB.Topic({
		ros : ros,
		name : box.params.pub_topic,
		messageType : "trajectory_msgs/JointTrajectory"
	});
	
	for (var i = 0; i < box.params.joints.length; i++) {
		joint = box.params.joints[i];
		
		var html = "";
		html += "<label for='" + joint.id + "_id'><span>" + joint.id.replace("_joint", "") + " [" + joint.keys[0] + "] &lt;" + joint.min.toFixed(2) + 
			"; </span> <span class='678to' id='" + joint.id + "_idSens'/>; " + joint.max.toFixed(2) + "&gt; <span>[" + joint.keys[1] + "]</span></label>";
		$(html).appendTo(element);
		$("#" + joint.id + "_id").trigger("create");
		
		hotkeys.addKey(new HotKey(joint.keys[0], function() {
			trigger_joint_change(this.obj.id, -this.obj.step, boxReference, topic);
		}, 
		null,
		joint));
		
		hotkeys.addKey(new HotKey(joint.keys[1], function() {
			trigger_joint_change(this.obj.id, +this.obj.step, boxReference, topic);
		},
		null,
		joint));
		
		var armStateTopic = new ROSLIB.Topic({
			ros : ros,
			name : '/joint_states',
			messageType : 'sensor_msgs/JointState'
		});
		armStateTopic.subscribe(function(message) {
			var value = 0;
			for (var i = 0; i < message.position.length; i++) {
				value = parseFloat(message.position[i]);
				setSliderValue(box.id, message.name[i], value)
			}
		});

	}

}

function trigger_joint_change(joint_id, change, boxReference, topic) {

	var joint_names = [];
	var points = [];
	var duration = { secs : 1, nsecs : 0};
	
	var box = boxesconfig.boxes[boxReference];
	for (var i = 0; i < box.params.joints.length; i++) {
		var this_joint_id = box.params.joints[i].id;
		if (this_joint_id == joint_id) { //only changed joint wil be published
			var this_joint_value = getSliderValue(box.id, this_joint_id);
			if (this_joint_value < 2 * 3.1416) { //it is valid value
				this_joint_value += change;
				if (this_joint_value < box.params.joints[i].min) 
					this_joint_value = box.params.joints[i].min
				if (this_joint_value > box.params.joints[i].max) 
					this_joint_value = box.params.joints[i].max
				points.push({ positions : [ this_joint_value ], effort : [50.0], time_from_start : duration });
				joint_names.push(this_joint_id);
			}
		}
	}

	var request = new ROSLIB.Message({
		joint_names: joint_names,
		points: points
	});
	topic.publish(request);
}

function getSliderValue(box_id, joint_id) {
	return parseFloat($("#" + box_id + " #" + joint_id + "_idSens").html());
}

function setSliderValue(box_id, joint_id, value) {
	$("#" + box_id + " #" + joint_id + "_idSens").html(value.toFixed(2));
}


