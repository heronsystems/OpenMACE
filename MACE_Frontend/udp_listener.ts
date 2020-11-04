var PORT = 8080;
var HOST = '127.0.0.1';

// var dgram = require('dgram');
// var server = dgram.createSocket('udp4');
var net = require('dgram');
var server = net.createSocket('udp4');

server.on('listening', function() {
  var address = server.address();
 console.log('UDP Server listening on ' + address.address + ':' + address.port);
});

server.on('message', function(message, remote) {
    console.log("New UDP Message: ")
    console.log(remote.address + ':' + remote.port +' - ' + message);
});

server.bind(PORT, HOST);