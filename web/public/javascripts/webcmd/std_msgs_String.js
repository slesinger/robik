

function std_msgs_String_onInit(element, box, boxReference) {

	var pub_topic = null;
	var sub_topic = null;
	
	var html = "";
	if (box.params.sub_topic !== undefined) {
		if (box.params.sub_topic.substr(0,1) == "<") { //html string only
			html += box.params.sub_topic;
		}
		else { //print string from topic stream
			"This feature is not implemented yet";
			sub_topic = null;
		}
	}
	
	if (box.params.pub_button_value === undefined) {
		box.params.pub_button_value = "Go";
	}

	if (box.params.pub_input_value === undefined) {
		box.params.pub_input_value = "";
	}

	if (box.params.prepend === undefined) {
		box.params.prepend = "";
	}

	if (box.params.prompt_label === undefined) {
		box.params.prompt_label = "";
	}

	if (box.params.pub_topic !== undefined) {
		pub_topic = new ROSLIB.Topic({
			ros : ros,
			name : box.params.pub_topic,
			messageType : "std_msgs/String"
		});
		var id = "std_msgs_String-" + box.id + "pub_button";
		//TODO include an input box, needs input change listener, likely a button to trigger publish
		html += "<div class='ui-mini'><input type='text' name='" + id + "' id='" + id + "' value='" + box.params.pub_input_value + "' placeholder='[" + box.params.key + "] " + box.params.prompt_label + "'/><button>" + box.params.pub_button_value + "</button></div>";
	}
	
	var key = "";
	if (box.params.key !== undefined) {
		//register key
		hotkeys.addKey(new HotKey(box.params.key, function() {
			var text = prompt(box.params.prompt_label);
			$( "#" + id).val(text);
			pub_string(box.params.prepend, text, pub_topic);
		}));
	}

	element.html("<div>" + html + "</div>");
	
	if (box.params.pub_topic !== undefined) {
		$( "#" + id).next().bind( "click", function(event, ui) {
			pub_string(box.params.prepend, $( "#" + id).val(), pub_topic);
		});
	}
}

function pub_string(prepend, text, topic) {
	var request = new ROSLIB.Message({
		data: prepend + text
	});
	topic.publish(request);

}