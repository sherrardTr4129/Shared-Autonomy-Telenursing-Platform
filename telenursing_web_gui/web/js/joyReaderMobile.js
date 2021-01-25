/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Socially Distanced Telenursing
 * Since: January 26th, 2021
 */

var Joy1 = new JoyStick('joy1');
var xyURL = "http://localhost:5000/xyJoyPost";
var xyData = {
	"x": 0,
	"y": 0
};

function sendXYData()
{
	$.ajax({type: 'POST',
		url: xyURL,
		data: JSON.stringify (xyData),
		success: function(data) {  },
		contentType: "application/json",
		dataType: 'json'
	});
}

setInterval(function(){ xyData.x=Joy1.GetX(); }, 50);
setInterval(function(){ xyData.y=Joy1.GetY(); }, 50);
setInterval(function(){ sendXYData() }, 200);

var Joy2 = new JoyStick('joy2');
var zURL = "http://localhost:5000/zJoyPost"
var zData = {
	"z": 0
}

function sendZData()
{
	$.ajax({type: 'POST',
		url: zURL,
		data: JSON.stringify (zData),
		success: function(data) {  },
		contentType: "application/json",
		dataType: 'json'
	});
}

setInterval(function(){ zData.z=Joy2.GetY(); }, 50);
setInterval(function(){ sendZData() }, 200);

