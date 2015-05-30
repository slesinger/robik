
function action_msgs_Button_onInit(element, box, boxReference) {

	var pub_topic = null;
	var sub_topic = null;

	var html = "";
	if (box.params.sub_topic !== undefined) {
			//"This feature is not implemented yet";
			sub_topic = null;
	}

	if (box.params.pub_topic !== undefined) {

		pub_topic = new ROSLIB.Topic({
			ros : ros,
			name : box.params.pub_topic,
			messageType : box.params.action_message_type
		});

		for (var i = 0; i < box.params.buttons.length; i++) {
			var button = box.params.buttons[i];
			var id = box.params.action_message_type.replace("/", "_") + i;
			var labelHotKey = '';
			if (box.params.buttons[i].key !== undefined) {
				labelHotKey = '[' + box.params.buttons[i].key + '] ';
			}
			html += "<div class='ui-btn-inline ui-mini'><input type='button' id='" + id + "' ord=" + i + " value='" + labelHotKey + button.label + "' /></div>";
		}

		element.html("<div>" + html + "</div>");

		for (var i = 0; i < box.params.buttons.length; i++) {
			var id = box.params.action_message_type.replace("/", "_") + i;
			$( "#" + id ).bind( "click", function(event, ui) {
				var request = new ROSLIB.Message({
					goal : boxesconfig.boxes[boxReference].params.buttons[event.srcElement.attributes.ord.value].goal
				});
				console.log(request);
				pub_topic.publish(request);
			});

			//register hot key
			if (box.params.buttons[i].key !== undefined) {
				hotkeys.addKey(new HotKey(box.params.buttons[i].key, function() {
					var request = new ROSLIB.Message({
//						goal : boxesconfig.boxes[boxReference].params.buttons[event.srcElement.attributes.ord.value].goal
							goal : this.obj.goal
					});
					console.log(request);
					pub_topic.publish(request);
				},
				function() {},
				box.params.buttons[i]
				));
			}
		}
	}
}
