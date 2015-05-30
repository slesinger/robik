
function sensor_msgs_Image_onInit(element, box, boxReference) {

	var pub_topic = null;
	var boxId = "mjpeg_" + box.id;
	var boxMsg = "mjpeg_" + box.id + "_msg";
	
	element.html("<div id='" + boxMsg + "'/><div id='" + boxId + "'></div>");
	
	if (box.params.sub_topic !== undefined) {
		var viewer = new MJPEGCANVAS.MultiStreamViewer({
			divID : 'mjpeg_' + box.id,
			host : boxesconfig.ros_host,
			width : box.params.width,
			height : box.params.height,
			quality : 70, //(1-100)
			topics : box.params.sub_topic,
			defaultStream : box.params.sub_topic[0]
		});
	}
	else {
		alert ("Mandatory attribute " + box.id + ".'params.pub_topic' is not configured");
	}
}

