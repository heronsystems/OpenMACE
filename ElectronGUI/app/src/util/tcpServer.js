net = require("net")

// Supports multiple client chat application

// Keep a pool of sockets ready for everyone
// Avoid dead sockets by responding to the 'end' event
var sockets = []

// Create a TCP socket listener
var s = net.Server(function(socket) {
    // Add the new client socket connection to the array of
    // sockets
    sockets.push(socket)

    // 'data' is an event that means that a message was just sent by the
    // client application
    socket.on("data", function(msg_sent) {
        // Loop through all of our sockets and send the data
        for (var i = 0; i < sockets.length; i++) {
            // Don't send the data back to the original sender
            // if (sockets[i] == socket) // don't send the message to yourself
            //     continue;
            // Write the msg sent by chat client
            sockets[i].write(msg_sent)
        }
    })
    // Use splice to get rid of the socket that is ending.
    // The 'end' event means tcp client has disconnected.
    socket.on("end", function() {
        var i = sockets.indexOf(socket)
        sockets.splice(i, 1)
    })
})

s.listen(1234)
console.log("System waiting at http://localhost:1234")
