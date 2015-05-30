
function std_msgs_Boolean_onInit(element, box, boxReference) {

	var pub_topic = null;
	var sub_topic = null;
	var html = "";
	var id = box.id + "_chk";

	if (box.params.sub_topic !== undefined) {
		sub_topic = new ROSLIB.Topic({
			ros : ros,
			name : box.params.sub_topic,
			messageType : box.params.sub_message
		});
		html += "<img id='" + id + "' src='/images/greylights.png'>";

		//pub and sub cannot be defined in same box
		if (box.params.pub_topic !== undefined)
			console.log("Error. pub_topic and sub_topic cannot be used together.");
	}
	
	if (box.params.pub_topic !== undefined && box.params.sub_topic === undefined) {
		// html += "<input type='checkbox' data-role='flipswitch' name='" + id + "' id='" + id + "'>";
		html += "<select name='" + id + "' id='" + id + "' data-role='flipswitch'><option value='off'>Off</option><option value='on'>On</option></select>";
	}

	element.html("<div id='" + id + "_container'>" + html + "</div>");

	if (box.params.sub_topic !== undefined) {
		var subscriber = {
			box: box,
			listener: function(message, box) {
				var id = box.id + "_chk";
				// console.log("#" + id + "  --  " + eval("message." + box.params.sub_mapping));
				var val = eval("message." + box.params.sub_mapping);
				var img = "/images/greylights.png";
				if (val == 'on' || val == 'true' || val == 1)
					img = "/images/greenlights.png";

				$("#" + id).attr("src", img);
			}
		};
		subTopics.subscribe(sub_topic, subscriber, box);
	}

	if (box.params.pub_topic !== undefined && box.params.sub_topic === undefined) {

		$("#" + id).val((box.params.initial_value == 1) ? "on" : "off");

		$("#" + id).bind( "change", function(event, ui) {
			bool_pub_string(box, (event.currentTarget.value == "on") ? 1 : 0);
		});
	}

}

function bool_pub_string(box, value) {
	var pub_topic = new ROSLIB.Topic({
		ros : ros,
		name : box.params.pub_topic,
		messageType : box.params.pub_message
	});
	
	var data = box.params.pub_data;

	if (data.header !== undefined) {
		data.header.stamp.secs = 1; //TODO how to populate
		data.header.stamp.nsecs = 0;
	}
	box.params.pub_mapping(data, value);

	var request = new ROSLIB.Message(data);
	pub_topic.publish(request);

}