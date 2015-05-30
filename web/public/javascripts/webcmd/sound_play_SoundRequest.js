function sound_play_SoundRequest_onInit(element, box, boxReference) {

	var pub_topic = null;

	if (box.params.pub_topic !== undefined) {

		pub_topic = new ROSLIB.Topic({
			ros : ros,
			name : "/" + box.params.pub_topic,
			messageType : box.params.action_message_type
		});

		var id = "sound_play_SoundRequest-" + box.params.pub_topic;
		var html = "<input type='text' id='" + id + "' value='Kočka leze dírou, pes oknem. Nebude-li pršet, nezmoknem.' /></br>";
		html += "<input type='button' id='button-" + id + "' value='Rekni' />";
		element.html("<div>" + html + "</div>");

		$( "#button-" + id ).bind( "click", function(event, ui) {
			var soundRequest = new ROSLIB.Message({
				sound : -3,
				command : 1,
				arg : this.previousSibling.previousSibling.value,
				arg2: "voice_czech_ph"
			});

			pub_topic.publish(soundRequest);
		});
	}

}