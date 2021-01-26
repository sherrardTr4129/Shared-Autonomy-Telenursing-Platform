/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Socially Distanced Telenursing
 * Since: January 26th, 2021
 */

var Joy3 = new JoyStick('joy3');
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

setInterval(function(){ xyData.x = Joy3.GetX(); }, 50);
setInterval(function(){ xyData.y = Joy3.GetY(); }, 50);
setInterval(function(){ sendXYData() }, 200);

var Joy4 = new JoyStick('joy4');
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

setInterval(function(){ zData.z=Joy4.GetY(); }, 50);
setInterval(function(){ sendZData() }, 200);

var Joy5 = new JoyStick('joy5');
var pitchYawURL = "http://localhost:5000/yawPitchJoyPost"
var pitchYawData = {
        "pitch": 0,
	"yaw": 0
}

function sendPitchYawData()
{
        $.ajax({type: 'POST',
                url: pitchYawURL,
                data: JSON.stringify (pitchYawData),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });
}

setInterval(function(){ pitchYawData.yaw = Joy5.GetY(); }, 50);
setInterval(function(){ pitchYawData.pitch = Joy5.GetX(); }, 50);
setInterval(function(){ sendPitchYawData() }, 200);

var Joy6 = new JoyStick('joy6');
var rollURL = "http://localhost:5000/rollJoyPost"
var rollData = {
        "roll": 0
}

function sendRollData()
{
        $.ajax({type: 'POST',
                url: rollURL,
                data: JSON.stringify (rollData),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });
}

setInterval(function(){ rollData.roll=Joy6.GetY(); }, 50);
setInterval(function(){ sendRollData() }, 200);

