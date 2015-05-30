

function nav2d_onInit(element, box, boxReference) {

	var move_base = null;
	var map_topic = null;
	var html = "";
	var id = box.id + "_nav2d";

	if (box.params.map_topic === undefined) {
		box.params.map_topic = '/map';
	}
	if (box.params.move_base === undefined) {
		box.params.move_base = '/move_base';
	}
	html += "<div id='"+ id + "'></div>";
	
	element.html("<div>" + html + "</div>");

	// Create the main viewer.
	var viewer = new ROS2D.Viewer({
		divID : id,
		width : 400,  //TODO calculate based on box size
		height : 400
	});

	// Setup the nav client.
	var nav = NAV2D.OccupancyGridClientNav({
		ros : ros,
		rootObject : viewer.scene,
		continuous: true,
		viewer : viewer,
		topic : box.params.map_topic,
		serverName : box.params.move_base
	});


}

