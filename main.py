from flask import Flask
from flask import Response
from flask import request

# from __future__ import division
# from __future__ import print_function
import numpy as np
import requests
import json
import urllib
import re
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import json

app = Flask(__name__)


    
def CVRP(addresses, API_key, vehicle_capacities, demands, starts, ends, depot, distance_type='Manhattan', use_1st_solu_strategy = False, scale=1): 
    '''return optimal solution for a Capacity Vehicle Routing Problem (CVRP) that 
    - each vehicle has limited capacity
    - allow dropped visits (but penalize_dropped visits)
    - allow setting different start and end locations for each vehicle
    
    Para
    ----
    - addresses: a list of real addresses (str) or (x, y)-locations
    #no longer need...num_vehicles: how many vehicles are used to cover those addresses
    - vehicle_capacities: a list of capacity of each vehicle
    - starts: a list of start-location-indices, each element for a vehicle/route
    - ends: a list of end-location-indices, each element for a vehicle/route
    - depot: the index of start & end address/location
      in the case that starts, ends, depot are all None, meaning we allow arbitary start and end location, we need to pre-process distance matrix by adding 1st row & 1st col all 0.
    - distance_type: 'Euclidean' or 'Manhattan'
    - use_1st_solu_strategy (bool)
    - scale (int): since the solver requires the distance_matrix has integer entries, we up-scale distance (says by 100) and take its integer part in order to keep the internal computation accurate.
    
    '''
    ###--------------------------------------------------------------------------<internal functions start>
    
    def create_data_model(distance_matrix, demands, vehicle_capacities, starts, ends, depot):
        '''create data model, a dictionary with following keys
        Para
        ----
        distance_matrix(a list of lists of numbers):entry (i,j) represents the distance bw ith row address and jth col address
        num_vehicles (int): number of vehicles in the fleet
        depot (int): start & end point for the trip represented by the index of addresses
        '''
        data = {}
        '''TBD
        if (starts is None) and (ends is None) and (depot == 0): # the case that allowing arbitary start and end location for each vehicle
            distance_matrix = [[0]*(len(distance_matrix)+1)] + [[0]+x for x in distance_matrix]
            demands = [0] + demands'''
        
        data['distance_matrix'] = distance_matrix
        data['demands'] = demands
        data['vehicle_capacities'] = vehicle_capacities
        data['num_vehicles'] = len(vehicle_capacities)
        data['starts'] = starts
        data['ends'] = ends
        data['depot'] = depot
        return data

    
    def print_solution(data, manager, routing, solution, scale):
        """Prints solution on console."""
        # Display dropped nodes.
        dropped_nodes = 'Dropped nodes:'
        for node in range(routing.Size()):
            if routing.IsStart(node) or routing.IsEnd(node):
                continue
            if solution.Value(routing.NextVar(node)) == node:
                dropped_nodes += ' {}'.format(manager.IndexToNode(node))
        print(dropped_nodes)
        # Display routes
        max_route_distance = 0
        total_distance = 0
        total_load = 0
        for vehicle_id in range(data['num_vehicles']):
            index = routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
            route_distance = 0
            route_load = 0
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                route_load += data['demands'][node_index]
                plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id)
            plan_output += ' {0} Load({1})\n'.format(manager.IndexToNode(index),
                                                     route_load)
            plan_output += 'Distance of the route: {}m\n'.format(route_distance/scale)
            plan_output += 'Load of the route: {}\n'.format(route_load)
            print(plan_output)
            max_route_distance = max(route_distance, max_route_distance)
            total_distance += route_distance
            total_load += route_load
        print('Total Distance of all routes: {}m'.format(total_distance/scale))
        print('Total Load of all routes: {}'.format(total_load))
        print('Maximum of the route distances: {}m'.format(max_route_distance/scale))



    def get_routes(manager, routing, solution):
        """Get vehicle routes from a solution and store them in an array (a list of lists)."""
        # Get vehicle routes and store them in a two dimensional array whose
        # i,j entry is the jth location visited by vehicle i along its route.
        routes = []
        for route_nbr in range(routing.vehicles()):
            index = routing.Start(route_nbr)
            route = [manager.IndexToNode(index)]
            while not routing.IsEnd(index):
                index = solution.Value(routing.NextVar(index))
                route.append(manager.IndexToNode(index))
            routes.append(route)
        return routes
    
    def send_request(origin_addresses, dest_addresses, API_key):
        """ Build and send request for the given origin and destination addresses."""
        def build_address_str(addresses):
            # Build a pipe-separated string of addresses
            address_str = ''
            for i in range(len(addresses) - 1):
                address_str += addresses[i] + '|'
            address_str += addresses[-1]
            return address_str

        request = 'https://maps.googleapis.com/maps/api/distancematrix/json?units=imperial'
        origin_address_str = build_address_str(origin_addresses)
        dest_address_str = build_address_str(dest_addresses)
        request = request + '&origins=' + origin_address_str + '&destinations=' + \
                           dest_address_str + '&key=' + API_key
        #replace this one that only works in python 2: jsonResult = urllib.urlopen(request).read()
        import urllib.request

        with urllib.request.urlopen(request) as url:
            jsonResult = url.read()
        response = json.loads(jsonResult)
        return response

    def build_distance_matrix(response):
        distance_matrix = []
        for row in response['rows']:
            row_list = [row['elements'][j]['distance']['value'] for j in range(len(row['elements']))]
            distance_matrix.append(row_list)
        return distance_matrix



    def addresses_to_distance_matrix(API_key, addresses):
        """turn addresses to distance matrix
        API_key: get one here https://developers.google.com/maps/documentation/distance-matrix/start#get-a-key
        addresses: a list of addresses
        """
        # Distance Matrix API only accepts 100 elements per request, so get rows in multiple requests.
        max_elements = 100
        num_addresses = len(addresses) # 16 in this example.
        # Maximum number of rows that can be computed per request (6 in this example).
        max_rows = max_elements // num_addresses
        # num_addresses = q * max_rows + r (q = 2 and r = 4 in this example).
        q, r = divmod(num_addresses, max_rows)
        dest_addresses = addresses
        distance_matrix = []
        # Send q requests, returning max_rows rows per request.
        for i in range(q):
            origin_addresses = addresses[i * max_rows: (i + 1) * max_rows]
            response = send_request(origin_addresses, dest_addresses, API_key)
            distance_matrix += build_distance_matrix(response)

        # Get the remaining remaining r rows, if necessary.
        if r > 0:
            origin_addresses = addresses[q * max_rows: q * max_rows + r]
            response = send_request(origin_addresses, dest_addresses, API_key)
            distance_matrix += build_distance_matrix(response)
        return distance_matrix


    def locations_to_distance_matrix(locations, scale, distance_type='Euclidean'):
        '''return the distance matrix between locations'''

        #scale each element in locations
        locations = [(p[0]*scale, p[1]*scale) for p in locations]

        if distance_type == 'Euclidean':
            dist_mat = [[int(((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**.5) for p2 in locations] for p1 in locations]

        if distance_type == 'Manhattan':
            dist_mat = [[abs(p1[0]-p2[0]) + abs(p1[1]-p2[1]) for p1 in locations] for p2 in locations]

        return dist_mat
    
    def preprocess_addresses(addresses):
        '''turn a string of separated words into a string of words connected by +'''
        return ['+'.join(re.sub("[^\w\s]", "", address).split()) for address in addresses]
    
    ###--------------------------------------------------------------------------<internal functions end>
    # create distance matrix
    if len(addresses[0])<3: # the case of (x, y) location
        distance_matrix = locations_to_distance_matrix(locations=addresses, scale=scale, distance_type=distance_type)
    else: # the case of address string
        addresses = preprocess_addresses(addresses)
        distance_matrix = addresses_to_distance_matrix(API_key, addresses)
    
    # create data
    data = create_data_model(distance_matrix, demands, vehicle_capacities, starts, ends, depot)
    
    # Create the routing index manager.
    #if (starts is not None) and (ends is not None):
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                               data['num_vehicles'], 
                                               data['starts'],
                                               data['ends'])
    """TBD
    
    else:
        manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                               data['num_vehicles'], 
                                               data['depot'])
    """
    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')
    
    # Allow to drop nodes.
    penalty = int(sum(sum(np.array(data['distance_matrix']))))
    for node in range(1, len(data['distance_matrix'])):
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    
    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        30000000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    if use_1st_solu_strategy:
        search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    else:
        search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        search_parameters.time_limit.seconds = 30
        search_parameters.log_search = True
        
    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution, scale)
        routs = get_routes(manager, routing, solution)
        return routs
    
# GET requests will be blocked
@app.route('/routes', methods=['POST'])
def json_example():
    request_data = request.get_json()
    addresses = request_data['addresses']
    num = request_data['num']
    vehicle_capacities = [100] * int(num)
    print(vehicle_capacities)
    num_vehicles = len(vehicle_capacities)
    # ['702 WEST JOHNSON STREET, SUITE 1101, MADISON', 
    #             '5801 S Ellis Ave, Chicago, IL 60637', 
    #             '38 Hillhouse Avenue New Haven, CT 06511',
    #             'North Ave NW, Atlanta, GA 30332',
    #             'Walt Disney World Resort 1375 E Buena Vista Dr Orlando, FL ',
    #             'Manhattan, NY 10036',
    #             '86 Brattle Street Cambridge, MA 02138',
    #             '450 Serra Mall, Stanford, CA 94305',
    #             '600 W College Ave, Tallahassee, FL 32306',
    #             '401 W Kennedy Blvd, Tampa, FL 33606',
    #             'Tempe, AZ 85281',
    #             'Los Angeles, CA 90095',
    #             '5998 Alcala Park, San Diego, CA 92110',
    #             'Tempe, AZ 85281',
    #             '801 Leroy Pl, Socorro, NM 87801',
    #             'Columbia, SC 29208',
    #             'Vancouver, BC V6T 1Z4, Canada'
    #             ]
    
    language = request_data['language']
    framework = request_data['framework']

    # two keys are needed because of the nested object
    python_version = request_data['version_info']['python']

    # an index is needed because of the array
    example = request_data['examples'][0]

    boolean_test = request_data['boolean_test']
    
    API_key = 'AIzaSyAaES4PwchuFSsn8xZ5LOhWyqvI5JzT7ng'

    demands = [0, 1, 1, 3, 6, 3, 6, 8, 8, 1, 2, 1, 2, 6, 6, 8, 8]

    starts = [0, 0, 0, 0]
    ends = [0, 0, 0, 0]
    depot = 0
    
    res = CVRP(addresses, API_key, vehicle_capacities, demands, starts, ends, depot, distance_type='Euclidean', use_1st_solu_strategy = False, scale=1000)    
    return Response(json.dumps(res),  mimetype='application/json')
    
    # return '''
    #        The language value is: {}
    #        The framework value is: {}
    #        The Python version is: {}
    #        The item at index 0 in the example list is: {}
    #        The boolean value is: {}'''.format(language, framework, python_version, example, boolean_test)
                   
@app.route('/')
def endPoint():
    # creating the data
    API_key = 'AIzaSyAaES4PwchuFSsn8xZ5LOhWyqvI5JzT7ng'
    addresses = ['702 WEST JOHNSON STREET, SUITE 1101, MADISON', 
                '5801 S Ellis Ave, Chicago, IL 60637', 
                '38 Hillhouse Avenue New Haven, CT 06511',
                'North Ave NW, Atlanta, GA 30332',
                'Walt Disney World Resort 1375 E Buena Vista Dr Orlando, FL ',
                'Manhattan, NY 10036',
                '86 Brattle Street Cambridge, MA 02138',
                '450 Serra Mall, Stanford, CA 94305',
                '600 W College Ave, Tallahassee, FL 32306',
                '401 W Kennedy Blvd, Tampa, FL 33606',
                'Tempe, AZ 85281',
                'Los Angeles, CA 90095',
                '5998 Alcala Park, San Diego, CA 92110',
                'Tempe, AZ 85281',
                '801 Leroy Pl, Socorro, NM 87801',
                'Columbia, SC 29208',
                'Vancouver, BC V6T 1Z4, Canada'
                ]
    demands = [0, 1, 1, 3, 6, 3, 6, 8, 8, 1, 2, 1, 2, 6, 6, 8, 8]
    vehicle_capacities = [12, 10, 13, 14]
    num_vehicles = len(vehicle_capacities)
    starts = [4, 2, 15, 16]
    ends = [0, 0, 0, 0]
    depot = 0
    ''' # addresses could be a list of (x, y) coordinates 
    addresses = [(0,0),
                (-2,4),
                (4,4),
                (-4,3),
                (-3,3),
                (1,2),
                (3,2),
                (-1,1),
                (2,1),
                (1,-1),
                (4,-1),
                (-3,-2),
                (-2,-2),
                (-1,-3),
                (2,-3),
                (-4,-4),
                (3,-4)]

    '''
    # res = CVRP(addresses, API_key, vehicle_capacities, demands, starts, ends, depot, distance_type='Euclidean', use_1st_solu_strategy = False, scale=1000)    
    # return Response(json.dumps(res),  mimetype='application/json')

    # return "Hello Alex!"

if __name__ == '__main__':
    app.run()