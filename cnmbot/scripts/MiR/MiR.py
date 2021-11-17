import requests
import json

from requests.api import post

# Get Request
ip = '192.168.12.20'
host = "http://" + ip +"/api/v2.0.0/"

#format the headers
headers = {}
headers['Content-Type']= 'application/json'
headers['Authorization'] = 'Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='

def post_mission(mission_name):
    """Function to use when you wish to post a mission to the queue
    Arguments:
        mission_name: the name you set in your Web Interface
    """
    get_missions = requests.get(host +'missions', headers = headers)
    response_native = json.loads(get_missions.text) #this is the missions in a list
    #obtai the mission ID for your interested mission from the list of missions
    for i in range(len(response_native)):
        if response_native[i]['name'] == mission_name:
            mission_id_temp = response_native[i]['guid']
        else:
            pass
    mission_id = {"mission_id":mission_id_temp}
    requests.post(host+'mission_queue', json = mission_id, headers = headers)

def delete_mission():
    """delete all the missions"""
    delete = requests.delete(host + 'mission_queue', headers = headers)

def check_completion():
    """check whether all the missions in the queue has completed or not"""
    status = False
    while status is False:
        check_mission_status = requests.get(host+'mission_queue', headers = headers)
        response_native = json.loads(check_mission_status.text)
        status_string = response_native[-1]['state']
        if status_string =='Done':
            status = True
        else:
            status = False


#post_mission("Docking_to_Charger")
#post_mission("Goto_N9")
#post_mission("Docking_to_Charger")
#post_mission("Goto_N9")
#post_mission("Docking_to_Charger")
post_mission("Goto_N9")
check_completion()
print("complete")