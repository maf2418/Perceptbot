// This function connects to the rosbridge server running on the local computer on port 9090
var rbServer = new ROSLIB.Ros({
    // url : 'ws://localhost:9090'
    url: 'ws://192.168.1.67:9090'
});

// This function is called upon the rosbridge connection event
rbServer.on('connection', function() {
    // Write appropriate message to #feedback div when successfully connected to rosbridge
    var fbDiv = document.getElementById('feedback');
    if (fbDiv) { fbDiv.innerHTML += "<p>Connected to websocket server.</p>"; }
    console.log("connected to websocket server");
});

// This function is called when there is an error attempting to connect to rosbridge
rbServer.on('error', function(error) {
    // Write appropriate message to #feedback div upon error when attempting to connect to rosbridge
    var fbDiv = document.getElementById('feedback');
    if (fbDiv) { fbDiv.innerHTML += "<p>Error connecting to websocket server.</p>"; }
    console.log("error connecting to websocket server");
});

// This function is called when the connection to rosbridge is closed
rbServer.on('close', function() {
    // Write appropriate message to #feedback div upon closing connection to rosbridge
    var fbDiv = document.getElementById('feedback');
    if (fbDiv) { fbDiv.innerHTML += "<p>Connection to websocket server closed.</p>"; }
    console.log("disconnected from websocket server");
});

// These lines create a topic object as defined by roslibjs
var cmdVelTopic = new ROSLIB.Topic({
    ros: rbServer,
    name: '/turtle1/cmd_vel',
    messageType: 'geometry_msgs/Twist'
});

// These lines create a message that conforms to the structure of the Twist defined in our ROS installation
// It initalizes all properties to zero. They will be set to appropriate values before we publish this message.
var twist = new ROSLIB.Message({
    linear: {
        x: 0.0,
        y: 0.0,
        z: 0.0
    },
    angular: {
        x: 0.0,
        y: 0.0,
        z: 0.0
    }
});


function pubMessageEnd() {

    twist.linear.x *= 0.2;
    twist.angular.z *= 0.2;
    console.log("published", twist.linear.x, twist.angular.z);
    cmdVelTopic.publish(twist);
    if (twist.linear.x > 0.01 && twist.angular.z < 0.01) {
        twist.linear.x = 0;
        twist.angular.z = 0;
    }
}

/* This function:
 - retrieves numeric values from the text boxes
 - assigns these values to the appropriate values in the twist message
 - publishes the message to the cmd_vel topic.
 */
function pubMessage(linearX, angularZ) {

    // Set the appropriate values on the message object
    twist.linear.x = linearX;
    twist.angular.z = angularZ;

    // Publish the message 
    console.log("published", twist.linear.x, twist.angular.z);
    cmdVelTopic.publish(twist);

}

setInterval(pubMessageEnd, 1000);