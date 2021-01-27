/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Socially Distanced Telenursing
 * Since: January 27th, 2021
 */

// make request to flask app to let it know the mobile control
// page has loaded and is active
var readyURL = "http://localhost:5000/mobilePageActive";
$(document).ready(function() {
  $.ajax({
    type: "POST",
    url: readyURL,
    data: { }
   });
});
