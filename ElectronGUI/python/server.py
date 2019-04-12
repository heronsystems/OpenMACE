from __future__ import print_function
import simplejson
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from SocketServer import ThreadingMixIn
from aircraftgc.ground_control import GroundControl
from waypoint_generation.gen_waypoints import gen_waypoints
from detect_serial_ports import detect_serial_ports

ground_control = GroundControl()


# Create handler
class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        path_split = self.path.split('/')
        del path_split[0]  # remove the first '/'
        if path_split[0] == 'connect':
            data = self.load_post_data()
            print("Recieved connect", data)
            ground_control.connect_to(data['comm_port'])
            self.wfile.write('true'.encode())
        elif path_split[0] == 'getHomeLocation':
            data = self.load_post_data()
            print("Recieved get home location", data)
            location = ground_control.get_home_location(data['comm_port'])
            self.wfile.write(simplejson.dumps(location))
        elif path_split[0] == 'getLocation':
            data = self.load_post_data()
            print("Recieved get location", data)
            location = ground_control.get_location(data['comm_port'])
            self.wfile.write(simplejson.dumps(location))
        elif path_split[0] == 'launch':
            data = self.load_post_data()
            print("Recieved takeoff", data)
            ground_control.takeoff(data['comm_port'])
            self.wfile.write('true'.encode())
        elif path_split[0] == 'land':
            data = self.load_post_data()
            print("Recieved land", data)
            ground_control.land(data['comm_port'])
            self.wfile.write('true'.encode())
        elif path_split[0] == 'guided':
            data = self.load_post_data()
            print("Recieved guided", data)
            ground_control.guided(data['comm_port'])
            self.wfile.write('true'.encode())
        elif path_split[0] == 'rtl':
            data = self.load_post_data()
            print("Recieved RTL", data)
            ground_control.RTL(data['comm_port'])
            self.wfile.write('true'.encode())
        elif path_split[0] == 'disconnect':
            data = self.load_post_data()
            print("Recieved disconnect", data)
            ground_control.disconnect(data['comm_port'])
            self.wfile.write('true'.encode())
        elif path_split[0] == 'sendWaypoints':
            data = self.load_post_data()
            ground_control.plan_mission(data['comm_port'], data['waypoints'])
            self.wfile.write('true'.encode())
        elif path_split[0] == 'generateWaypoints':
            data = self.load_post_data()
            planning_regions, vehicle_paths = gen_waypoints(data)
            dic = {"planningRegions": planning_regions, "vehiclePaths": vehicle_paths}
            self.wfile.write(simplejson.dumps(dic).encode())
        elif path_split[0] == 'detectSerialPorts':
            self.send_headers()
            self.wfile.write(simplejson.dumps(detect_serial_ports()))
        else:
            self.wfile.write('Malformed request'.encode())

    def do_GET(self):
        """ Handle GET Request"""
        path_split = self.path.split('/')
        del path_split[0]  # remove the first '/'

    def load_post_data(self):
        self.send_headers()
        post_bytes = self.rfile.read(int(self.headers['Content-Length']))
        jsondata = simplejson.loads(post_bytes)
        return jsondata

    def send_headers(self, content_type='application/json'):
        self.send_response(200)
        self.send_header('Content-type', content_type)
        self.end_headers()


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""


if __name__ == '__main__':
    PORT = 10081

    # Port on which server will run.
    HTTPDaemon = ThreadedHTTPServer(('', PORT), Handler)
    # HTTPDaemon = HTTPServer(('', PORT), Handler)
    print("Listening at port", PORT)

    try:
        HTTPDaemon.serve_forever()
    except KeyboardInterrupt:
        pass

    HTTPDaemon.server_close()
    print("Server stopped")
