var twist;
var cmdVel;
var publishImmidiately = true;
var robot_IP;
var manager;
var teleop;
var ros;
VIDEO_LOADED = false;

function moveAction(linear, angular) {
    if (linear !== undefined && angular !== undefined) {
        twist.linear.x = linear;
        twist.angular.z = angular;
    } else {
        twist.linear.x = 0;
        twist.angular.z = 0;
    }
    cmdVel.publish(twist);
}

function initVelocityPublisher() {
    // Init message with zero values.
    twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: 0
        }
    });
    // Init topic object
    cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: "/cmd_vel",
        messageType: "geometry_msgs/Twist"
    });
    // Register publisher within ROS system
    cmdVel.advertise();
}

function initTeleopKeyboard() {
    // Use w, s, a, d keys to drive your robot

    // Check if keyboard controller was aready created
    if (teleop == null) {
        // Initialize the teleop.
        teleop = new KEYBOARDTELEOP.Teleop({
            ros: ros,
            topic: "/cmd_vel"
        });
    }

    // Add event listener for slider moves
    robotSpeedRange = document.getElementById("robot-speed");
    robotSpeedRange.oninput = function() {
        teleop.scale = robotSpeedRange.value / 100;
    };
}

function createJoystick() {
    // Check if joystick was aready created
    if (manager == null) {
        joystickContainer = document.getElementById("joystick");
        // joystck configuration, if you want to adjust joystick, refer to:
        // https://yoannmoinet.github.io/nipplejs/
        var options = {
            zone: joystickContainer,
            position: { left: 50 + "%", top: 105 + "px" },
            mode: "static",
            size: 200,
            color: "#0066ff",
            restJoystick: true
        };
        manager = nipplejs.create(options);
        // event listener for joystick move
        manager.on("move", function(evt, nipple) {
            // nipplejs returns direction is screen coordiantes
            // we need to rotate it, that dragging towards screen top will move robot forward
            var direction = nipple.angle.degree - 90;
            if (direction > 180) {
                direction = -(450 - nipple.angle.degree);
            }
            // convert angles to radians and scale linear and angular speed
            // adjust if youwant robot to drvie faster or slower
            var lin = Math.cos(direction / 57.29) * nipple.distance * 0.005;
            var ang = Math.sin(direction / 57.29) * nipple.distance * 0.05;
            // nipplejs is triggering events when joystic moves each pixel
            // we need delay between consecutive messege publications to
            // prevent system from being flooded by messages
            // events triggered earlier than 50ms after last publication will be dropped
            if (publishImmidiately) {
                publishImmidiately = false;
                moveAction(lin, ang);
                setTimeout(function() {
                    publishImmidiately = true;
                }, 50);
            }
        });
        // event litener for joystick release, always send stop message
        manager.on("end", function() {
            moveAction(0, 0);
        });
    }
}

window.onload = function() {
    // determine robot address automatically
    // robot_IP = location.hostname;
    // set robot address statically
    laptop_IP = "192.168.1.96";
    robot_IP = "192.168.1.67"; // is static for PerceptBot
    // // Init handle for rosbridge_websocket
    ros = new ROSLIB.Ros({
        url: "ws://" + robot_IP + ":9090" // is this the right socket on the PerceptBot?
    });

    initVelocityPublisher();
    // get handle for video placeholder

    video = document.getElementById('video');
    try {
        if (true) { // if no error was found ...
            // Populate video source 
            // video.src = "http://localhost:8080/stream_viewer?topic=/usb_cam/image_raw";
            video.src = "http://" + laptop_IP + ":8080/stream?topic=/image_color";
        }
        VIDEO_LOADED = true
    } catch (err) {
        console.log('ERROR loading video, check connection to laptop_IP', err)
    }
    if (VIDEO_LOADED) {
        video.onload = function() {
            // joystick and keyboard controls will be available only when video is correctly loaded

            createJoystick();
            initTeleopKeyboard();
        };
        video.onerror = (e) => {
            console.log('ERROR loading video, check connection to Rosbridge, Robot_IP, laptop_IP', e);
            //video.src = "./control/chrome.webm";
        };
    }
}
window.onerror = (e) => {
    console.log('ERROR connecting to laptop or server IP, configuration needed.', e);
}