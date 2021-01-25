/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Socially Distanced Telenursing
 * Since: January 26th, 2021
 */

var Joy3 = new JoyStick('joy3');
var yawPitchURL = "http://localhost:5000/yawPitchJoyPost";
var yawPitchData = {
        "yaw": 0,
        "pitch": 0
};

function sendYawPitchData()
{
        $.ajax({type: 'POST',
                url: yawPitchURL,
                data: JSON.stringify (yawPitchData),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });
}

setInterval(function(){ yawPitchData.pitch = Joy3.GetX(); }, 50);
setInterval(function(){ yawPitchData.yaw = Joy3.GetY(); }, 50);
setInterval(function(){ sendYawPitchData() }, 200);
setInterval(function(){ console.log(yawPitchData) }, 200);

var Joy4 = new JoyStick('joy4');
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

setInterval(function(){ rollData.roll=Joy4.GetY(); }, 50);
setInterval(function(){ sendRollData() }, 200);

var Joy5 = new JoyStick('joy5');
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

setInterval(function(){ rollData.roll=Joy5.GetY(); }, 50);
setInterval(function(){ sendRollData() }, 200);

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

setInterval(function(){ rollData.roll=Joy5.GetY(); }, 50);
setInterval(function(){ sendRollData() }, 200);

