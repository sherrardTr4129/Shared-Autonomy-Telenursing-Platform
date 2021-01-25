/* Author: Trevor Sherrard
 * Course: Human Robot Interaction
 * Project: Autonomous Camera Control Optimization
 * Since: November 1st, 2020
 */

function openFunction()
{
	openURL = "http://localhost:5000/openGripper"
	openData = {"val": 1};
	$.ajax({type: 'POST',
		url: openURL,
		data: JSON.stringify (openData),
		success: function(data) {  },
		contentType: "application/json",
		dataType: 'json'
	});
}

function closeFunction()
{
        closeURL = "http://localhost:5000/closeGripper"
        closeData = {"val": 1};
        $.ajax({type: 'POST',
                url: closeURL,
                data: JSON.stringify (closeData),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });
}
