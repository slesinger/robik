

function urdf3d_onInit(element, box, boxReference) {

	var move_base = null;
	var map_topic = null;
	var html = "";
	var id = box.id + "_urdf3d";

	if (box.params.map_topic === undefined) {
		box.params.map_topic = '/map';
	}
	if (box.params.move_base === undefined) {
		box.params.move_base = '/move_base';
	}
	html += "<div id='"+ id + "'></div>";
	
	element.html("<div>" + html + "</div>");

	// Create the main viewer.
	var viewer = new ROS3D.Viewer({
		divID : id,
		width : 400,  //TODO calculate based on box size
		height : 400,
		antialias : true
	});

	//Add grid
	viewer.addObject(new ROS3D.Grid());

	//Setup a client to listen to TFs.
	var tfClient = new ROSLIB.TFClient({
		ros : ros,
		angularThres : 0.01,
		transThres : 0.01,
		rate : 10.0
	});

	// Setup the map client.
	var gridClient = new ROS3D.OccupancyGridClient({
		ros : ros,
		rootObject : viewer.scene,
		topic : map_topic,
		tfClient : tfClient,
		continuous : true
	});

	// Setup the URDF client.
	var urdfClient = ROS3D.UrdfClient({
		ros : ros,
		path : 'http://robik:3000/',
		tfClient : tfClient,
		rootObject : viewer.scene
	});

}

