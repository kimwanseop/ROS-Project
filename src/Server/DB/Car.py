from DB.DB_Controller import ASAP_DB

class Car():
    def __init__(self, brand:str, car_name:str, type:str, car_number:str, pin_number:str, img_path:str = None):
        self.car_number = car_number
        self.brand = brand
        self.car_name = car_name
        self.type = type
        self.pin_number = pin_number

        self.battery = 0
        self.destroied = False
        self.isrented = False

        self.img_path = img_path
        self.pos = None



class CarDB(ASAP_DB):
    def __init__(self):
        super().__init__()
        self.car_dict = {}
        self.car_brand = {}
        self.car_name = {}
        self.car_type = {}

    def add_car(self, car:Car):
        if car.car_number not in self.car_dict:
            self.car_dict[car.car_number] = car
        
            if car.brand not in self.car_brand:
                self.car_brand[car.brand] = [car.car_number]
            else:
                self.car_brand[car.brand].append(car.car_number)
            if car.car_name not in self.car_name:
                self.car_name[car.car_name] = [car.car_number]
            else:
                self.car_name[car.car_name].append(car.car_number)
            if car.type not in self.car_type:
                self.car_type[car.type] = [car.car_number]
            else:
                self.car_type[car.type].append(car.car_number)

            value = f"'{car.car_number}', '{car.brand}', '{car.car_name}', '{car.type}', '{car.pin_number}', '{car.battery}', '{car.destroied}', '{car.isrented}'"
            print('car_info', value)
        
    def get_stat_info(self, info_type='all'):
        
        if info_type == 'all':
            data = self.car_name
        elif info_type == 'brand':
            data = self.car_brand
        elif info_type == 'type':
            data = self.car_type
        else:
            return Exception('Invalid info_type')


        return data, self.car_name

    def remove_car(self, car:Car):
        if car.car_number in self.car_dict:
            self.car_brand[car.brand].remove(car.car_number)
            self.car_name[car.car_name].remove(car.car_number)
            del(self.car_dict[car.car_number])
            if len(self.car_name[car.car_name])==0:
                del(self.car_name[car.car_name])
            if len(self.car_brand[car.brand])==0:
                del(self.car_brand[car.brand])

        
