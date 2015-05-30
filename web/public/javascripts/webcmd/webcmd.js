
var boxSize = 110;

var hotkeys = {
	keys: [],
	addKey: function (hotkey) {
		this.keys[hotkey.keycode] = hotkey;
	},
	getKey: function (key) {
		return this.keys[key.charCodeAt(0)];
	}
};

var subTopics = {
	topics: [],
	subscribers: [],
	subscribe: function (topic, subscriber, box) {
		this.topics[topic.name] = topic;

		var subs = this.subscribers[topic.name];
		if (subs === undefined) {
			subs = new Array();
			topic.subscribe(function(message) {
				subTopics.setData(topic, message);
			});

		}
		subs.push(subscriber);
		this.subscribers[topic.name] = subs;
		console.log(subscriber.box);
	},
	getTopic: function (name) {
		return this.topics[name];
	},
	getData: function (name) {
		return this.data[name];
	},
	setData: function (topic, message) {
		var subs = this.subscribers[topic.name];
		subs.forEach(function(subscriber){
			subscriber.listener(message, subscriber.box);
		});

	}

};

var ros;
var ros_param;

//see http://javascript.info/tutorial/keyboard-events
function HotKey(key, actionDown, actionUp, obj) {
	this.keycode = key.charCodeAt(0);
	this.obj = obj;
	if (actionDown === undefined) actionDown = null;
	if (actionUp === undefined) actionUp = null;
	this.actionDown = actionDown;
	this.actionUp = actionUp;
}

function handleKeyDown(keycode) {
	var k = hotkeys.keys[keycode];

	if (k === undefined) {
		console.log("Unrecognized key '" + String.fromCharCode(keycode) + "' (" + keycode + ")");
	}

	if (k !== undefined && k.actionDown != null) {
		k.actionDown();
	}
}

function handleKeyUp(keycode) {
	var k = hotkeys.keys[keycode];

	if (k === undefined) {
		console.log("Unrecognized key '" + String.fromCharCode(keycode) + "' (" + keycode + ")");
	}

	if (k !== undefined && k.actionUp != null) {
		k.actionUp();
	}
}

function webcmd_init() {

	if (boxesconfig.ros_host === undefined)
		boxesconfig.ros_host = "127.0.0.1";

	if (boxesconfig.ros_port === undefined)
		boxesconfig.ros_port = "9090";

	boxesconfig.ros_url = "ws://" + boxesconfig.ros_host + ":" + boxesconfig.ros_port;

	// Connecting to ROS.
	ros = new ROSLIB.Ros({
		url : boxesconfig.ros_url
	});
	ros_param = new ROSLIB.Param({
		ros : ros,
		name : 'robot_description'
	});

	// Init boxes
	for (var bi = 0; bi < boxesconfig.boxes.length; bi++) {
		var box = boxesconfig.boxes[bi];
		console.log("Init " + box.id + ((box.disabled == true) ? " ...is disabled!" : ""));

		if (box.disabled == true)
			continue;

		//defaults
		if (box.width === undefined)
			box.width = 1;
		if (box.height === undefined)
			box.height = 1;
		if (box.label === undefined)
			box.label = box.id;


		//visuals
		var html = "<div class='box_wrapper' id='" + box.id + "'><div class='box_label_outter'><div class='box_label'>" + box.label + "</div></div><div class='box_content " + box.widget_type.replace("/", "_") + "'></div><div>";
		$(html).appendTo(".ui-content");
		$("#" + box.id).css({
			'width': (box.width * boxSize - 6) + 'px',
			'height': (box.height * boxSize - 6) + 'px',
			'background-color': box.color
		});

		//onInit hooks
		html += window[box.widget_type.replace("/", "_") + "_onInit"]($("#" + box.id + " .box_content"), box, bi);
		$("#" + box.id + " .box_content").trigger("create");
	}

	//register key listeners
	var body = document.getElementsByTagName('body')[0];
	body.addEventListener('keydown', function(e) {
		handleKeyDown(e.keyCode, true);
	}, false);
	body.addEventListener('keyup', function(e) {
		handleKeyUp(e.keyCode, true);
	}, false);
}
