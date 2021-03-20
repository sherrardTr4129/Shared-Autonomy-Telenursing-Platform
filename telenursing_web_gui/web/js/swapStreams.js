function swapStreams(){
	var swapURL = 
	$.ajax({type: 'POST',
		url: frURL,
		data: JSON.stringify (frData),
		success: function(data) {  },
		contentType: "application/json",
		dataType: 'json'
	});
}

}
