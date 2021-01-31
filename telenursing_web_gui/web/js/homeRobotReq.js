/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Socially Distanced Telenursing
 * Since: January 31st, 2021
 */

homeRobotURL = "http://localhost:5000/makeRobotHomeReq"
function sendHomeReq()
{
	$.get(homeRobotURL, function(data, status){});
}
