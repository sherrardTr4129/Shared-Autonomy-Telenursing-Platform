/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Socially Distanced Telenursing
 * Since: January 26th, 2021
 */

var Joy1 = new JoyStick('joy1');
var frURL = "http://localhost:5000/fwdRevJoyPost";
var frData = {
	"FwdRev": 0,
};

function sendFRData()
{
	$.ajax({type: 'POST',
		url: frURL,
		data: JSON.stringify (frData),
		success: function(data) {  },
		contentType: "application/json",
		dataType: 'json'
	});
}

setInterval(function(){ frData.FwdRev=Joy1.GetY(); }, 50);
setInterval(function(){ sendFRData() }, 200);
setInterval(function(){ console.log(frData) }, 200);

var Joy2 = new JoyStick('joy2');
var spinURL = "http://localhost:5000/spinJoyPost"
var spinData = {
	"spin": 0
}

function sendSpinData()
{
	$.ajax({type: 'POST',
		url: spinURL,
		data: JSON.stringify (spinData),
		success: function(data) {  },
		contentType: "application/json",
		dataType: 'json'
	});
}

setInterval(function(){ spinData.spin=Joy2.GetX(); }, 50);
setInterval(function(){ sendSpinData() }, 200);
setInterval(function(){ console.log(spinData) }, 200);

