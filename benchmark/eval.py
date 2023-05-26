import time
class Trace:
    def __init__(self) -> None:
        self.traces = [] 
    
    def update_trace(self, function_name: str, input: str, output: str, old_state: str, new_state: str) -> None:
        state_dictionary = {
            "function_name" : function_name,
            "input" : input,
            "output" : output,
            "old_state": old_state,
            "new_state": new_state
        }
        self.traces.append(state_dictionary)

    def __str__(self):
        output = ""
        for idx, trace in enumerate(self.traces):
            msg = "idx {}: function name: {}, input: {}, output: {} \n old_state: {} \n new_state: {}".format(idx, 
                                        trace["function_name"], trace["input"], trace["output"], trace["old_state"], trace["new_state"])
            output = "\n".join([output, msg])
        return output

class Entity:
    def __init__(self, name, type, response) -> None:
        self.name = name
        self.response = response
        self.type = type

    def get_asked(self, question: str, options: list[str]):
        assert(self.type == "human")
        for option in options:
            if option == self.response:
                return option        
        assert(False)
    
    def say(self, msg: str):
        assert(type(msg) == str)
        assert(self.type == "robot")

class World:
    def __init__(self, state: dict, trace: Trace, transition: dict) -> None:
        self.state = state
        self.trace = trace 
        self.transition = transition
        self.transition_count = 0
    
    def match_label():
        pass 

    def get_location(self, entity_name: str) -> str :
        # helper function
        for k,v in self.state.items():
            for name in v.keys():
                if entity_name == name:
                    return k 
        raise "cannot get location for {}".format(entity_name)            

    def get_entity(self, entity_name: str) -> Entity :
        # helper function
        for k,v in self.state.items():
            for name in v.keys():
                if entity_name == name:
                    return v[name]
        raise "cannot get entity for {}".format(entity_name)

    def move_entity(self, entity_name, entity_new_location):
        entity_old_location = self.get_location(entity_name)
        entity = self.state[entity_old_location][entity_name]
        del self.state[entity_old_location][entity_name]
        self.state[entity_new_location][entity_name] = entity

    def transition_hook(self):
        # check the state of the world, use the state to change the world 
        self.transition_count += 1
        if self.transition.get(str(self.transition_count)):
            entity = self.transition[str(self.transition_count)][0]
            location = self.transition[str(self.transition_count)][1]
            self.move_entity(entity, location)

    def wrap_state(self):
        state_dict = {}
        for k in self.state.keys():
            state_dict[k] = []
            for name in self.state[k].keys():
                state_dict[k].append(name)
        return state_dict

    # ======================================================
    def handle_get_current_location(self) -> str :
        self.transition_hook()
        return self.get_location("robot")
    
    def handle_get_all_rooms(self) -> list[str] :
        self.transition_hook()
        return list(self.state.keys())
    
    def handle_is_in_room(self, object: str) -> bool :
        robot_location = self.get_location("robot")
        object_location = self.get_location(object)
        self.transition_hook()
        return robot_location == object_location
    
    def handle_go_to(self, location : str) -> None :
        old_state = self.wrap_state()
        self.move_entity("robot", location)
        new_state = self.wrap_state()

        self.trace.update_trace("go", location, str(None), str(old_state), str(new_state))
        self.transition_hook()

    def handle_ask(self, person : str, question : str, options: list[str]) -> str :
        old_state = self.wrap_state()
        robot_location = self.get_location("robot")
        person_location = self.get_location(person)
        assert(robot_location == person_location)
        self.transition_hook()
        
        person_entity = self.get_entity(person)
        response = person_entity.get_asked(question, options)
        new_state = self.wrap_state()
        
        self.trace.update_trace("ask", question, str(response), str(old_state), str(new_state))
        self.transition_hook()
        return response
    
    def handle_say(self, message : str) -> None :
        old_state = self.wrap_state()
        robot_entity = self.get_entity("robot")
        robot_entity.say(message)
        new_state = self.wrap_state()

        self.trace.update_trace("say", message, str(None), str(old_state), str(new_state))
        self.transition_hook()

def get_current_location() -> str :
    global WORLD
    return WORLD.handle_get_current_location()

def get_all_rooms() -> list[str] :
    global WORLD
    return WORLD.handle_get_all_rooms()

def is_in_room(object : str) -> bool :
    global WORLD
    return WORLD.handle_is_in_room(object)

def go_to(location : str) -> None :
    global WORLD
    WORLD.handle_go_to(location)

def ask(person : str, question : str, options: list[str]) -> str :
    global WORLD
    return WORLD.handle_ask(person, question, options)

def say(message : str) -> None :
    global WORLD
    WORLD.handle_say(message)

def run_program(program : str) -> None :
    exec(program)

if __name__ == "__main__":
    state = {
        "living room" : {
            "robot" : Entity("robot", "robot", None),
        },
        "kitchen" : {},
        "bedroom": {"zichao" : Entity("zichao", "human", "Good")}
    }
    transition = {
        "10" : ["zichao", "kitchen"]
    }
    trace = Trace()

    WORLD = World(state, trace, transition)


    program = "my_rooms = get_all_rooms()\n" + \
          "for r in my_rooms:\n" + \
          "  go_to(r)\n" + \
          "  say(f\"I am in {r}\")\n"
    
    program = "list_of_rooms = get_all_rooms()\nstart_loc = get_current_location()\ncandies = [\"Chocolate\", \"Gummies\", \"Licorice\"]\ncandies_count = {candy : 0 for candy in candies}\nfor room in list_of_rooms:\n    if \"office\" not in room:\n        continue\n    go_to(room)\n    if is_in_room(\"person\"):\n        response = ask(\"Person\", \"Which kind of candy would you like?\", candies)\n        candies_count[response] += 1\ngo_to(start_loc)\nfor candy, count in candies_count.items():\n    say(\"We need to buy \" + str(count) + \" \" + candy)"
    
    program = """start_loc = get_current_location()
go_to("kitchen")
while True:
    if is_in_room("zichao"):
        response = ask("zichao", "How are you doing?", ["Good", "Bad"])
        break
    time.sleep(1)
go_to(start_loc)
say("zichao said: " + response)
    """

    run_program(program)

    print(WORLD.trace)