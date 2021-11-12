import requests
import json

from requests.api import head

# Get Request
ip = '192.168.12.20'
host = "http://" + ip +"/api/v2.0.0/"

#format the headers
headers = {}
headers['Content-Type']= 'application/json'
headers['Authorization'] = 'Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='

print(headers)

get_missions = requests.get(host +'missions', headers = headers)
print(get_missions.text)

mission_id = {"mission_id":"4d20cfdb-2d08-11ec-b470-00012978ede1"}
post_mission = requests.post(host+'mission_queue', json = mission_id, headers = headers)

#delete = requests.delete(host + 'mission_queue', headers = headers)