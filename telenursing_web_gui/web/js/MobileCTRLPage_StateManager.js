/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Socially Distanced Telenursing
 * Since: January 27th, 2021
 */


// URLs for interfacing with Flask app
var getStateURL = "http://localhost:5000/getPrimaryStream";

// primary and secondary stream URL holders
var primaryURLString = "";
var secondaryURLString = "";

// primary and secondary stream URL holders for previously
// seen URL string
var lastPrimaryString = "";
var lastSecondaryString = "";

// boolean to check if it's the first time in the function
var first = true;

// This function will get the current primary and secondary stream
// state from the flask app, and see if it has changed.
function pollBackendForChange(){
	$.get(getStateURL, function(data, status){
		console.log(data)
		var jsonObj = JSON.parse(JSON.stringify(data));
		var primaryURLString = jsonObj.primaryStream;
		var secondaryURLString = jsonObj.secondaryStream;
		
		if(first)
		{
			// set initial iframe URLs
			document.getElementById('mainPagePrimaryStream').src = primaryURLString;
			document.getElementById('mainPageSecondaryStream').src = secondaryURLString;

			// set last URL string values
			lastPrimaryString = primaryURLString;
			lastSecondaryString = secondaryURLString;

			first = false;
		}
		else
		{
			if(primaryURLString != lastPrimaryString && 
				secondaryURLString != lastSecondaryString){
				// URLs have been changed on back end, change values within iframe
				document.getElementById('mainPagePrimaryStream').src = primaryURLString;
                        	document.getElementById('mainPageSecondaryStream').src = secondaryURLString;
			}
			// update last URL string values
			lastPrimaryString = primaryURLString;
			lastSecondaryString = secondaryURLString;
		}
	});
}

setInterval(function(){ pollBackendForChange() }, 200);

