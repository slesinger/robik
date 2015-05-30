/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * Manages connection to the server and all interactions with ROS.
 *
 * Emits the following events:
 *   * 'change' - emitted with a change in speed occurs
 *
 * @constructor
 * @param options - possible keys include:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the Twist topic to publish to, like '/arm_twist'
 *   * throttle (optional) - a constant throttle for the speed
 */
KEYBOARDTELEOP.TeleopArm = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/arm_twist';
  // permanent throttle
  var throttle = options.throttle || 1.0;

  // used to externally throttle the speed (e.g., from a slider)
  this.scale = 1.0;

  // linear x and y movement and angular z movement
  var arm_clamp = 500;
  var arm_roll = 500;
  var arm_elbow = 500;
  var arm_shoulder = 500;
  var arm_yaw = 500;

  var armTwist = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'trajectory_msgs/JointTrajectory' //pokud pouziva topic uz existujiciho publikovaneho message, tak se typem podridi
  });

  // sets up a key listener on the page used for keyboard teleoperation
  var handleKey = function(keyCode, keyDown) {
    // used to check for changes in speed
    oldArm_clamp = arm_clamp;
    oldArm_roll = arm_roll;
    oldArm_elbow = arm_elbow;
    oldArm_shoulder = arm_shoulder;
    oldArm_yaw = arm_yaw;

	var myJointNames = [];
	var myPoints = [];

    var speed = 0;
    // throttle the speed by the slider and throttle constant
    if (keyDown === true) {
      speed = throttle * that.scale;
    }
    var duration = { secs : 0, nsecs : 500};
    // check which key was pressed
    var ourKey = true;
    switch (keyCode) {
      case 66:
        // B yaw left
        arm_yaw = normalize(arm_yaw + (-0.5 * speed));
        myJointNames.push("yaw");
        myPoints.push({ positions : [ arm_yaw ], time_from_start : duration });
        break;
      case 78:
        // N yaw right
        arm_yaw = normalize(arm_yaw + (0.5 * speed));
        myJointNames.push("yaw");
        myPoints.push({ positions : [ arm_yaw ], time_from_start : duration });
        break;
      case 84:
        // T shoulder up
        arm_shoulder = normalize(arm_shoulder + (0.3 * speed));
        myJointNames.push("shoulder");
        myPoints.push({ positions : [ arm_shoulder ], time_from_start : duration });
        break;
      case 71:
        // G shoulder down
        arm_shoulder = normalize(arm_shoulder + (-0.3 * speed));
        myJointNames.push("shoulder");
        myPoints.push({ positions : [ arm_shoulder ], time_from_start : duration });
        break;
      case 89:
        // Y elbow up
        arm_elbow = normalize(arm_elbow + (0.4 * speed));
        myJointNames.push("elbow");
        myPoints.push({ positions : [ arm_elbow ], time_from_start : duration });
        break;
      case 72:
        // H elbow down
        arm_elbow = normalize(arm_elbow + (-0.4 * speed));
        myJointNames.push("elbow");
        myPoints.push({ positions : [ arm_elbow ], time_from_start : duration });
        break;
      case 77:
        // M roll left
        arm_roll = normalize(arm_roll + (0.5 * speed));
        myJointNames.push("roll");
        myPoints.push({ positions : [ arm_roll ], time_from_start : duration });
        break;
      case 188:
        // , roll right
        arm_roll = normalize(arm_roll + (-0.5 * speed));
        myJointNames.push("roll");
        myPoints.push({ positions : [ arm_roll ], time_from_start : duration });
        break;
      case 74:
        // J clamp close
        arm_clamp = normalize(arm_clamp + (1 * speed));
        myJointNames.push("clamp");
        myPoints.push({ positions : [ arm_clamp ], time_from_start : duration });
        break;
      case 75:
        // K clamp open
        arm_clamp = normalize(arm_clamp + (-1 * speed));
        myJointNames.push("clamp");
        myPoints.push({ positions : [ arm_clamp ], time_from_start : duration });
        break;
      default:
        ourKey = false;
        //console.log("unknown key " + keyCode);
    }

	if (ourKey == true) {
    // publish the command
    var twist = new ROSLIB.Message({
    	joint_names : myJointNames,
    	points : myPoints
    });
    armTwist.publish(twist);

    // check for changes
    if (oldArm_clamp !== arm_clamp || oldArm_roll !== arm_roll || oldArm_elbow !== arm_elbow || oldArm_shoulder !== arm_shoulder || oldArm_yaw !== arm_yaw) {
	that.emit('change', twist);
	$("#arm_yaw").html(arm_yaw);
	$("#arm_shoulder").html(arm_shoulder);
	$("#arm_elbow").html(arm_elbow);
	$("#arm_roll").html(arm_roll);
	$("#arm_clamp").html(arm_clamp);
    }
    }
  };

  // handle the key
  var body = document.getElementsByTagName('body')[0];
  body.addEventListener('keydown', function(e) {
    handleKey(e.keyCode, true);
  }, false);
  body.addEventListener('keyup', function(e) {
    handleKey(e.keyCode, false);
  }, false);
};
KEYBOARDTELEOP.TeleopArm.prototype.__proto__ = EventEmitter2.prototype;

function normalize(val) {
	var x = Math.floor(val);
	if (x == 0) 
		return 0;
	if (x < -1000) x = -1000;
	if (x > 1000) x = 1000;
	console.log(x + "|" + val);
	return x;
}
