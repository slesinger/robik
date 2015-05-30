
function sensor_msgs_JointState_onInit(element, box, boxReference) {

	var topic = new ROSLIB.Topic({
		ros : ros,
		name : box.params.pub_topic,
		messageType : "trajectory_msgs/JointTrajectory"
	});
	
	for (var i = 0; i < box.params.joints.length; i++) {
		joint = box.params.joints[i];
		
		var html = "";
		html += "<label for='" + joint.id + "_id'>" + joint.id + "</label>";
		html += "<span>[" + joint.keys[0] + "]</span> <input type='range' name='" + joint.id + "_id' id='" + joint.id + "_id' value='" + joint.min + "' min='" + (joint.min * 1000) + "' max='" + (joint.max * 1000) + "' > <span>[" + joint.keys[1] + "]</span>";
		$(html).appendTo(element);
		$("#" + joint.id + "_id").change(function() {
			trigger_joint_change_2(joint.id, boxReference, topic);
		});
		
		hotkeys.addKey(new HotKey(joint.keys[0], function() {
				var newVal = getSliderValue_2(this.obj.id) - this.obj.step;
				setSliderValue_2(this.obj.id, newVal);
				trigger_joint_change_2(this.obj.id, boxReference, topic);
			}, 
			null,
			joint)
		);
		
		hotkeys.addKey(new HotKey(joint.keys[1], function() {
				setSliderValue_2(this.obj.id, getSliderValue_2(this.obj.id) + this.obj.step);
				trigger_joint_change_2(this.obj.id, boxReference, topic);
			},
			null,
			joint)
		);

	}

}

function trigger_joint_change_2(joint_id, boxReference, topic) {

	var names = [];
	var positions = [];
	
	var box = boxesconfig.boxes[boxReference];
	for (var i = 0; i < box.params.joints.length; i++) {
		names.push(box.params.joints[i].id);
		positions.push(getSliderValue_2(box.params.joints[i].id));
	}

	var request = new ROSLIB.Message({
		name: names,
		position: positions
	});
	topic.publish(request);
}

function getSliderValue_2(joint_id) {
	return $("#" + joint_id + "_id").val() / 1000.0;
}

function setSliderValue_2(joint_id, value) {
	$("#" + joint_id + "_id").val(value * 1000.0);
}


