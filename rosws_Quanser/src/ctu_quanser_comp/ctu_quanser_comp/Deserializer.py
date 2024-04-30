import pickle

class Deserializer:
    def serialize_message(message):
        try:
            message = pickle.dumps(message)
            return message
        except pickle.PicklingError as e:
            print(f"Error occurred while serializing message: {e}")
            return None

    def deserialize_message(message):
        try:
            message = pickle.loads(message)
            return message
        except pickle.UnpicklingError as e:
            print(f"Error occurred while deserializing message: {e}")
            return None
