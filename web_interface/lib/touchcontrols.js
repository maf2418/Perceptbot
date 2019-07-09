
// ---------------------------------------------------------------------------------------------
// this section contains basic functions for data stored as X,Y Points and X1,Y1,X2,Y2 Boxes
// ---------------------------------------------------------------------------------------------

var Box = function (x, y, w, h) {
	this.x1 = Math.round (x || 0);
	this.y1 = Math.round (y || 0);
	this.x2 = Math.round ((x + w) || 0);
	this.y2 = Math.round ((y + h) || 0);
	this.width = this.x2 - this.x1;
	this.height = this.y2 - this.y1;
	this.center = new Point (x + (w/2), y + (h/2));
};

Box.prototype = {
reset : function ( x, y, w, h ) {
			this.x1 = Math.round(x);
			this.y1 = Math.round(y);
			this.x2 = Math.round(x + w);
			this.y2 = Math.round(y + h);
			this.width = this.x2 - this.x1;
			this.height = this.y2 - this.y1;
			this.center.reset (this.x1 + (this.width/2), this.y1 + (this.height / 2));
			return this;
		},

toString : function (decPlaces) {
			decPlaces = decPlaces || 3; 
			var scalar = Math.pow (10,decPlaces); 
			return "[" +
					Math.round (this.x1 * scalar) / scalar + "," + Math.round (this.y1 * scalar) / scalar +
					" , " +
					Math.round (this.x2 * scalar) / scalar + "," + Math.round (this.y2 * scalar) / scalar +
					"]";
		},

clone : function () {
			return new Box (this.x1, this.y1, this.x2 - this.x1, this.y2 - this.y1);
		},

copyTo : function (b) {
			b.x1 = this.x1;
			b.y1 = this.y1;
			b.x2 = this.x2;
			b.y2 = this.y2;
			b.width = this.width;
			b.height = this.height;
			b.center.copyFrom(this.center);
		},

copyFrom : function (b) {
			this.x1 = b.x1;
			this.y1 = b.y1;
			this.x2 = b.x2;
			this.y2 = b.y2;
			this.width = b.width;
			this.height = b.height;
			this.center.copyFrom(b.center);
		},	

move : function (v) {
			this.x1 += v.x;
			this.y1 += v.y;
			this.x2 += v.x;
			this.y2 += v.y;
			this.center.reset (this.x1 + (this.width/2), this.y1 + (this.height / 2));
			return this; 
		},

enlarge : function (v) {
			this.x2 += v.x;
			this.y2 += v.y;
			this.width = this.x2 - this.x1;
			this.height = this.y2 - this.y1;
			this.center.reset (this.x1 + (this.width/2), this.y1 + (this.height / 2));
		},

inside : function (v) {
			return ((this.x1 <= v.x) && (this.y1 <= v.y) &&
					(this.x2 >= v.x) && (this.y2 >= v.y))
		}
};


var Point = function (x,y) {
	this.x = Math.round(x || 0); 
	this.y = Math.round(y || 0); 
};


Point.prototype = {
reset : function ( x, y ) {
			this.x = Math.round(x);
			this.y = Math.round(y);
			return this;
		},

toString : function (decPlaces) {
			decPlaces = decPlaces || 3; 
			var scalar = Math.pow(10,decPlaces); 
			return "[" + Math.round (this.x * scalar) / scalar + ", " + Math.round (this.y * scalar) / scalar + "]";
		},

clone : function () {
			return new Point(this.x, this.y);
		},

copyTo : function (v) {
			v.x = this.x;
			v.y = this.y;
		},

copyFrom : function (v) {
			this.x = v.x;
			this.y = v.y;
		},	

equals : function (v) {
			return((this.x == v.x) && (this.y == v.y));
		}
};




// ---------------------------------------------------------------------------------------------
// this section contains random functions
// ---------------------------------------------------------------------------------------------

function is_touch_device() {
	return (('ontouchstart' in window)
			|| (navigator.MaxTouchPoints > 0)
			|| (navigator.msMaxTouchPoints > 0));
}




// ---------------------------------------------------------------------------------------------
// this section UI target functions
// ---------------------------------------------------------------------------------------------

var TouchTarget = function (name, type) {
	// public properties
	// this are read/write and take effect following thier change
	// the display does not automatically refresh when these are changed
	this.uiColor = '#aaaaaa';
	this.touchColor = '#cccccc';
	this.autoCenter = false;
	this.roundRect = false;

	// protected properties
	// these are read only
	// changes make to these are unpredictable and not tested
	this.controlCallback = null;
	this.elementName = name;
	this.uiType = type;	// "joystick", "vslider", "hslider", "cross", "round", "square", "left", "up", "right", "down"
	this.deltaX = 0;
	this.deltaY = 0;
	this.isActive = false;
	
	// private properties
	// these are only used internally
	this._id = -1;
	this._origin  = new Point (0, 0);
	this._initial = new Point (0, 0);
	this._current = new Point (0, 0);
	this._box = new Box (0, 0, 0, 0);
	this._element = el = document.getElementById(name);
	this._ui_canvas = null;
	this._touch_canvas = null;
	this._ui_ctx = null;
	this._touch_ctx = null;
	this._touch_enabled = is_touch_device();

	var self = this;
	if (el) {
		// the ui control canvas must be created before the touch
		// overlay so the latter will be 'on top' and receives events
		
		this._ui_canvas = document.createElement( 'canvas' );
		this._ui_canvas.setAttribute ("style", "position: absolute;");

		el.appendChild(this._ui_canvas);
		// Make it visually fill the positioned parent
		//this._ui_canvas.style.width ='100%';
		//this._ui_canvas.style.height='100%';
		this._ui_ctx = this._ui_canvas.getContext('2d');

		this._touch_canvas = document.createElement( 'canvas' );
		this._touch_canvas.setAttribute ("style", "position: absolute;");
		el.appendChild(this._touch_canvas);	
		// Make it visually fill the positioned parent
		//this._touch_canvas.style.width ='100%';
		//this._touch_canvas.style.height='100%';
		this._touch_ctx = this._touch_canvas.getContext('2d');
		
		this.resizeControl_handler(null, self);
		
		if(is_touch_device()) {
		// we make a local reference to 'this' so it will still be in
		// context during the event handler callback
			this._touch_canvas.addEventListener( 'touchstart', function(e) { self.onTouchStart_handler(e, self); }, false );
			this._touch_canvas.addEventListener( 'touchmove',  function(e) { self.onTouchMove_handler(e, self);  }, false );
			this._touch_canvas.addEventListener( 'touchend',   function(e) { self.onTouchEnd_handler(e, self);   }, false );

			window.addEventListener( 'orientationchange', function (e) { self.resizeControl_handler(e, self); }, false );
			window.addEventListener( 'resize',            function (e) { self.resizeControl_handler(e, self); }, false );
		}
	} else {
		console.log ("addTouchControl: unrecognized element id - " + name);
	}
}


TouchTarget.prototype = {
setCallback : function (cb) {
				  this.uiCallback = cb;
			  },

drawControl : function() {
				  // ctx is the 2d canvas
				  var c = this._ui_ctx;
				  c.clearRect(this._box.x1,this._box.y1,this._touch_canvas.width, this._touch_canvas.height);

				  // setup some assumed defaults
				  c.globalAlpha = 1.0;
				  c.strokeStyle = this.uiColor;
				  c.fillStyle = this.uiColor;
				  c.lineWidth = 2;

				  var center = this._box.center;
				  var radius = Math.round (Math.min (this._box.width, this._box.height) / 2) - 1;

				  switch (this.uiType) {
					  case "joystick":
						  /* the joystick ui display consists of an out circle, a bold inner circle,
						   * and a center 'hub'. since the joystick 'knob' will extend
						   * beyond the perimeter by half its diameter, we reduce all of the ui control by
						   * the same amount. the 'knob' is 1/4 the overall diameter of the ui control
						   * itself.
						   */

						  radius = Math.round (radius * 0.75);
						  
						  // normal outer circle
						  c.beginPath();
						  c.arc(center.x, center.y, radius, 0, Math.PI*2, true); 
						  c.stroke();
						  // bold inner circle
						  c.beginPath(); 
						  c.lineWidth = 6;
						  c.arc(center.x, center.y, radius * 0.8,0,Math.PI*2,true); 
						  c.stroke();
						  c.lineWidth = 2;
						  // filled inner circle
						  c.beginPath(); 
						  c.globalAlpha = 0.2;
						  c.arc(center.x, center.y, radius * 0.8,0,Math.PI*2,true); 
						  c.fill();
						  c.globalAlpha = 1.0;
						  // filled center circle
						  c.beginPath(); 
						  c.arc(center.x, center.y, radius *0.2,0,Math.PI*2,true); 
						  c.fill();
						  break;
					  case "vslider":
						  if (this.roundRect) {
							  c.beginPath();
							  c.arc(center.x, this._box.y2 - radius - 1, radius, 0, Math.PI, false); 
							  c.stroke();
							  c.globalAlpha = 0.2;
							  c.fill();
							  c.globalAlpha = 1.0;

							  c.beginPath();
							  c.arc(center.x, this._box.y1 + radius + 1, radius, 0, Math.PI, true);
							  c.stroke();
							  c.globalAlpha = 0.2;
							  c.fill();
							  c.globalAlpha = 1.0;

							  // arcs render a bit thicker than lines so we fatten up the lines
							  c.lineWidth = 3;
							  c.beginPath();
							  c.moveTo(this._box.x1, this._box.y1 + radius);
							  c.lineTo(this._box.x1, this._box.y2 - radius);
							  c.moveTo(this._box.x2, this._box.y1 + radius);
							  c.lineTo(this._box.x2, this._box.y2 - radius);
							  c.stroke();
							  c.globalAlpha = 0.2;
							  c.rect(this._box.x1, this._box.y1 + radius + 1, this._box.width, this._box.height - (radius * 2) - 2);
							  c.fill();
						  }
						  else {
							  c.lineWidth = 3;
							  c.beginPath();
							  c.rect(this._box.x1, this._box.y1, this._box.width, this._box.height);
							  c.stroke();
							  c.globalAlpha = 0.2;
							  c.fill();
						  }
						  break;
					  case "hslider":
						  if (this.roundRect) {
							  c.beginPath();
							  c.arc(this._box.x2 - radius - 1, center.y, radius, Math.PI * 1.5, Math.PI * 0.5, false); 
							  c.stroke();
							  c.globalAlpha = 0.2;
							  c.fill();
							  c.globalAlpha = 1.0;

							  c.beginPath();
							  c.arc(this._box.x1 + radius + 1, center.y, radius, Math.PI * 1.5, Math.PI * 0.5, true);
							  c.stroke();
							  c.globalAlpha = 0.2;
							  c.fill();
							  c.globalAlpha = 1.0;

							  // arcs render a bit thicker than lines so we fatten up the lines
							  c.lineWidth = 3;
							  c.beginPath();
							  c.moveTo(this._box.x1 + radius, this._box.y1);
							  c.lineTo(this._box.x2 - radius, this._box.y1);
							  c.moveTo(this._box.x1 + radius, this._box.y2);
							  c.lineTo(this._box.x2 - radius, this._box.y2);
							  c.stroke();
							  c.globalAlpha = 0.2;
							  c.rect(this._box.x1 + radius + 1, this._box.y1, this._box.width - (radius * 2) - 2, this._box.height);
							  c.fill();
						  }
						  else {
							  c.lineWidth = 3;
							  c.beginPath();
							  c.rect(this._box.x1, this._box.y1, this._box.width, this._box.height);
							  c.stroke();
							  c.globalAlpha = 0.2;
							  c.fill();
							  c.globalAlpha = 1.0;
						  }
						  break;
					  case "round":
						  c.beginPath(); 
						  c.arc(center.x, center.y, radius,0,Math.PI*2,true); 
						  c.stroke();
						  break;
					  case "square":
						  c.beginPath();
						  c.rect(0, 0, this._box.width, this._box.height);
						  c.stroke();
						  break;
					  case "up":
						  c.beginPath(); 
						  c.moveTo(this._box.center.x, this._box.y1);
						  c.lineTo(this._box.center.x + (this._box.width / 2), this._box.y2);
						  c.lineTo(this._box.center.x - (this._box.width / 2), this._box.y2);
						  c.closePath();
						  c.stroke();
						  break;
					  case "down":
						  c.beginPath(); 
						  c.moveTo(this._box.center.x, this._box.y2);
						  c.lineTo(this._box.center.x + (this._box.width / 2), this._box.y1);
						  c.lineTo(this._box.center.x - (this._box.width / 2), this._box.y1);
						  c.closePath();
						  c.stroke();
						  break;
					  case "left":
						  c.beginPath(); 
						  c.moveTo(this._box.x1, this._box.center.y);
						  c.lineTo(this._box.x2, this._box.center.y + (this._box.height / 2));
						  c.lineTo(this._box.x2, this._box.center.y - (this._box.height / 2));
						  c.closePath();
						  c.stroke();
						  break;
					  case "right":
						  c.beginPath(); 
						  c.moveTo(this._box.x2, this._box.center.y);
						  c.lineTo(this._box.x1, this._box.center.y + (this._box.height / 2));
						  c.lineTo(this._box.x1, this._box.center.y - (this._box.height / 2));
						  c.closePath();
						  c.stroke();
						  break;
					  default:
						  // display red box with X
						  console.log ("addTouchControl: unrecognized type - " + this.uiType);
						  c.beginPath();
						  c.strokeStyle = "red";
						  c.strokeRect(0, 0, this._box.width, this._box.height);
						  c.moveTo(0, 0)
						  c.lineTo(this._box.width, this._box.height);
						  c.moveTo(this._box.width, 0);
						  c.lineTo(0, this._box.height);
						  c.stroke();
				  }
				  if (!this._touch_enabled) {
					  c.fillStyle	 = "white"; 
					  c.fillText("no touch enabled device detected\n", 10, 10); 
				  }
				  this.drawTouch();
			  },
			  
drawTouch : function() {
				  // ctx is the 2d canvas
				  var c = this._touch_ctx;
				  c.clearRect(this._box.x1,this._box.y1,this._touch_canvas.width, this._touch_canvas.height);

				  c.strokeStyle = c.fillStyle = this.touchColor;
				  c.lineWidth = 1;

				  /* for buttons, we key off of 'deltas' to determine if button is active. we
				   * render the touch of an inactive button at 20% alpha and active at 100%
				   */
				  var buttonColor = (this.isActive ? this.touchColor : this.uiColor);
				  var center = this._box.center;
				  var radius = Math.round (Math.min (this._box.width, this._box.height) / 2) - 1;

				  switch (this.uiType) {
					  case "joystick":
						  /* the joystick touch display consists of thick line eminating from the center of
						   * the ui out to the touch point with a filled circle centered on the touch point
						   * as a facsimile of a 'knob'. the trick is to keep the 'knob' within the
						   * bounding box of the ui. we do this by temporarily translating and scaling the
						   * canvase area.
						   */
						  var radius = Math.round (radius * 0.25);

						  c.save();
						  {
							  c.translate (radius, radius);
							  c.scale (0.75, 0.75);

							  c.beginPath();
							  c.arc(this._current.x, this._current.y, radius,0,Math.PI*2,true); 
							  c.fill();

							  c.beginPath(); 
							  c.lineWidth = 12;
							  c.lineCap = "round";
							  c.moveTo(this._box.center.x, this._box.center.y);
							  c.lineTo(this._current.x, this._current.y);
							  c.stroke();
						  }
						  c.restore();
						  break;
					  case "vslider":
						  // draw a fill square but do daw outside of the ui area
						  c.beginPath();
						  if (this.roundRect) {
							  c.arc(this._box.center.x, this._current.y, radius-2, 0, Math.PI*2, true);
							  c.fill();
						  }
						  else
							  c.fillRect(this._box.x1+3, this._current.y - Math.round(this._box.width / 2) + 3,
										 this._box.width-6, this._box.width-6);
						  break;
					  case "hslider":
						  // draw a fill square but do daw outside of the ui area
						  c.beginPath();
						  if (this.roundRect) {
							  c.arc(this._current.x, this._box.center.y, radius-2, 0, Math.PI*2, true);
							  c.fill();
						  }
						  else
							  c.fillRect(this._current.x - Math.round(this._box.height / 2) + 3, this._box.y1+3,
										 this._box.height-6, this._box.height-6);
						  break;
					  case "round":
						  c.strokeStyle = c.fillStyle = buttonColor;
						  // c.globalAlpha = 0.4;
						  c.beginPath(); 
						  c.arc(center.x, center.y, radius-2,0,Math.PI*2,true); 
						  c.fill();
						  break;
					  case "square":
						  c.strokeStyle = c.fillStyle = buttonColor;
						  c.beginPath(); 
						  c.fillRect(2, 2, this._box.width - 4, this._box.height - 4);
						  break;
					  case "up":
						  c.strokeStyle = c.fillStyle = buttonColor;
						  c.beginPath(); 
						  c.moveTo(this._box.center.x, this._box.y1 + 4);
						  c.lineTo(this._box.center.x + (this._box.width / 2) - 3, this._box.y2 - 2);
						  c.lineTo(this._box.center.x - (this._box.width / 2) + 3, this._box.y2 - 2);
						  c.fill();
						  break;
					  case "down":
						  c.strokeStyle = c.fillStyle = buttonColor;
						  c.beginPath(); 
						  c.moveTo(this._box.center.x, this._box.y2 - 4);
						  c.lineTo(this._box.center.x + (this._box.width / 2) - 3, this._box.y1 + 2);
						  c.lineTo(this._box.center.x - (this._box.width / 2) + 3, this._box.y1 + 2);
						  c.fill();
						  break;
					  case "left":
						  c.strokeStyle = c.fillStyle = buttonColor;
						  c.beginPath(); 
						  c.moveTo(this._box.x1 + 4, this._box.center.y);
						  c.lineTo(this._box.x2 - 2, this._box.center.y + (this._box.height / 2) - 3);
						  c.lineTo(this._box.x2 - 2, this._box.center.y - (this._box.height / 2) + 3);
						  c.fill();
						  break;
					  case "right":
						  c.strokeStyle = c.fillStyle = buttonColor;
						  c.beginPath(); 
						  c.moveTo(this._box.x2 - 4, this._box.center.y);
						  c.lineTo(this._box.x1 + 2, this._box.center.y + (this._box.height / 2) - 3);
						  c.lineTo(this._box.x1 + 2, this._box.center.y - (this._box.height / 2) + 3);
						  c.fill();
						  break;
					  default:
						  // uh oh
						  // display touch details above and to the right of the touch point
						  c.beginPath();
						  c.fillStyle = "white";
						  c.fillText(this._elementId + " id : "+this._id+" x:"+this._current.x+" y:"+this._current.y, 20, -40);
				  }
			  },

updateDeltas : function() {
				  // update deltaX and deltaY for the UI type
				  var x = this._current.x;
				  var y = this._current.y;

				  var deltaX = 0;
				  var deltaY = 0;
				  switch (this.uiType) {
					  case "joystick":
						  deltaX = ((x - this._box.x1) / (this._box.width  / 2)) - 1.0;
						  deltaY = ((this._box.y2 - y) / (this._box.height / 2)) - 1.0;
						  break;
					  case "vslider":
						  deltaX = 0;
						  deltaY = (((this._box.y2 - (this._box.width / 2)) - y) / ((this._box.height - this._box.width)  / 2)) - 1.0;
						  break;
					  case "hslider":
						  deltaX = ((x - (this._box.x1 + (this._box.height / 2))) / ((this._box.width - this._box.height)  / 2)) - 1.0;
						  deltaY = 0;
						  break;
					  case "round":
					  case "square":
					  case "left":
					  case "up":
					  case "right":
					  case "down":
						  if (this.isActive)
							  deltaX = deltaY = 1.0;
						  break;
					  default:
						// uh oh
				  }
				  if ((this.deltaX != deltaX) || (this.deltaY != deltaY)) {
					  this.deltaX = deltaX;
					  this.deltaY = deltaY;

					  if (this.uiCallback)
						  this.uiCallback(this, deltaX, deltaY);

					  return true;
				  }

				  return false;
			  },


/*	
 *	Touch event (e) properties : 
 *	e.touches: 			Array of touch objects for every finger currently touching the screen
 *	e.targetTouches: 	Array of touch objects for every finger touching the screen that
 *						originally touched down on the DOM object the transmitted the event.
 *	e.changedTouches	Array of touch objects for touches that are changed for this event. 					
 *						I'm not sure if this would ever be a list of more than one, but would 
 *						be bad to assume. 
 *
 *	Touch objects : 
 *
 *	identifier: An identifying number, unique to each touch event
 *	target: DOM object that broadcast the event
 *	clientX: X coordinate of touch relative to the viewport (excludes scroll offset)
 *	clientY: Y coordinate of touch relative to the viewport (excludes scroll offset)
 *	screenX: Relative to the screen
 *	screenY: Relative to the screen
 *	pageX: Relative to the full page (includes scrolling)
 *	pageY: Relative to the full page (includes scrolling)
 */	

onTouchStart_handler : function (e, self) {
				  for(var i = 0; i<e.changedTouches.length; i++){
					  var touch = e.changedTouches[i]; 

					  // determine if touch is within this UI control
					  for(var i=0; i<touchTargets.length; i++) {
						  var x = touch.clientX - self._origin.x;
						  var y = touch.clientY - self._origin.y;
						  var pt = new Point (x, y);
						  if (self._box.inside(pt)) {
							  if (self._id < 0) {
								  self._id = touch.identifier;
								  self.isActive = true;
								  // we set the starting position to be the center of the UI widget 
								  self._initial.copyFrom(self._box.center);
								  self._current.reset(x, y);
								  self.onTouchMove_handler (e, self);
								  self.drawTouch();
							  } 		
							  break;
						  }
					  }
				  }
			  },

onTouchMove_handler : function(e, self) {
				  // Prevent the browser from doing its default thing (scroll, zoom)
				  e.preventDefault();

				  for(var i = 0; i<e.changedTouches.length; i++){
					  var touch = e.changedTouches[i]; 

					  // determine if touch target has moved and update it
					  if (self._id == touch.identifier) {
						  var x = touch.clientX - self._origin.x;
						  var y = touch.clientY - self._origin.y;

						  switch (self.uiType) {
							  case "joystick":
								  x = Math.max (Math.min (x, self._box.x2), self._box.x1);
								  y = Math.max (Math.min (y, self._box.y2), self._box.y1);
								  break;
							  case "vslider":
								  x = self._box.center.x;
								  y = Math.max (Math.min (y, (self._box.y2 - (self._box.width / 2))), (self._box.y1 + (self._box.width / 2)));
								  break;
							  case "hslider":
								  x = Math.max (Math.min (x, (self._box.x2 - (self._box.height / 2))), (self._box.x1 + (self._box.height / 2)));
								  y = self._box.center.y;
								  break;
							  case "round":
								  break;
							  case "square":
								  break;
							  case "lefttriangle":
								  break;
							  case "uptriangle":
								  break;
							  case "rughttriangle":
								  break;
							  case "downtriangle":
								  break;
							  default:
								  // uh oh
						  }
						  
						  self._current.reset(x, y);
						  if (self.updateDeltas())
							  self.drawTouch();
						  // console.log (self.elementName + ' (' + x + ', ' + y + ')');
						  break;
					  }
				  }
			  },

onTouchEnd_handler : function(e, self) {
				  for(var i = 0; i<e.changedTouches.length; i++){
					  var touch = e.changedTouches[i];

					  // determine if touch is this control
					  if (self._id == touch.identifier) {
						  self._id = -1;
						  self.isActive = false;
						  // auto-center controls as needed
						  if (self.autoCenter)
							  self._current.copyFrom (self._box.center);

						  if (self.updateDeltas())
							  self.drawTouch();
						  break;
					  }
				  }
			  },

resizeControl_handler : function (e, self) {
				  var el = self._element;
				  if (el) {
					  // setup the screen real estate for the ui control
					  //this._box.reset (el.offsetLeft, el.offsetTop, el.offsetWidth, el.offsetHeight);
					  var viewportOffset = el.getBoundingClientRect();
					  self._origin.reset (viewportOffset.left, viewportOffset.top);
					  self._box.reset (0, 0, el.offsetWidth, el.offsetHeight);
					  self._initial.reset (self._box.center.x, self._box.center.y);
					  self._current.reset (self._box.center.x, self._box.center.y);
					  // ... set the internal size to match
					  self._touch_canvas.width  = self._ui_canvas.width  = el.offsetWidth;
					  self._touch_canvas.height = self._ui_canvas.height = el.offsetHeight;
				  }
				  window.scrollTo(0,0);
				  self.drawControl();
			  }
};


// ---------------------------------------------------------------------------------------------
// this section contains internal variables.
// These should eventually be encapsulated into an object
// ---------------------------------------------------------------------------------------------

var touchTargets = []; // array of UI controls



// ---------------------------------------------------------------------------------------------
// this section contains the public API
// ---------------------------------------------------------------------------------------------

function addTouchControl (name, type, args)
{
	var ui = new TouchTarget (name, type);
	touchTargets.push (ui);

	if (args) {
		for (var parm in args) {
			if (args.hasOwnProperty(parm)) {
				switch (parm) {
					case 'uiColor':
						ui.uiColor = args[parm];
						break;
					case 'touchColor':
						ui.touchColor = args[parm];
						break;
					case 'autoCenter':
						ui.autoCenter = args[parm];
						break;
					case 'roundRect':
						ui.roundRect = args[parm];
						break;
					case 'uiCallback':
						ui.uiCallback = args[parm];
						break;
					default:
						console.log ("addTouchControl: unrecognized parameter - " + parm);
						break;
				}
			}
		}
		// if the colors changed, then the control will need to be redrawn
		ui.drawControl();
	}
	
	return ui;
}

function initTouchControls()
{
	// this is a place holder in case there is need in teh future
	var touchable = is_touch_device();

	return touchable;
}
