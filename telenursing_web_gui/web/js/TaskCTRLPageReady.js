/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Socially Distanced Telenursing
 * Since: January 27th, 2021
 */

var readyURL = "http://localhost:5000/taskPageActive";
$(document).ready(function() {
  $.ajax({
    type: "POST",
    url: readyURL,
    data: { }
   });
});

