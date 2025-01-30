import numpy as np 
from DB.DB_Controller import ASAP_DB

class Member():
    def __init__(self, name, phone, license, img_path):
        self.name = name
        self.phone = phone
        self.license = license 
        self.img_path = img_path 

        self.member_code = None
        self.member_grade = 'normal'
        self.point = 0

        self.rent_cost = {} # 탑승 시간 : 랜트 시간, 반납 시간, 비용
        self.car_acident = [] # 사고 발생 시간 
        self.is_renting = 0

        self.ID = None 
        self.PW = None 

    def generate_code(self):
        code = ''
        for _ in range(8):
            code += str(np.random.randint(0, 10))
        return code

class MemberDB(ASAP_DB):
    def __init__(self):
        super().__init__()
        self.member_dict = {} # 회원 번호 : person
        self.name = {} # 회원 이름 : 회원 번호
        self.IDPW = {}
        self.init_db()

    def add_member(self, member:Member):
        while True :
            member_code = member.generate_code()
            if member_code not in self.member_dict:
                member.member_code = member_code
                self.update_data(member, member_code)
                break
        columns = "name, phone, license, img_path, member_code, point, is_renting, ID, PW"
        values = f"'{member.name}', '{member.phone}', '{member.license}', '{member.img_path}', '{member.member_code}', '{member.point}', '{member.is_renting}', '{member.ID}', '{member.PW}'"
        self.insert_values('person', columns, values)

    def update_data(self, member:Member, member_code):
        self.member_dict[member_code] = member
        self.IDPW[member.ID] = member.PW
        print(self.IDPW)
        if member.name not in self.name:
            self.name[member.name] = [member_code]
        else:
            self.name[member.name].append(member_code)

    def del_member(self, member:Member):
        if member.member_code in self.member_dict:
            del(self.member_dict[member.member_code])
            if len(self.name[member.name])==0:
                del(self.name[member.name])
            else:
                self.name[member.name].remove(member.member_code)

    def init_db(self):
        data = self.get_data('person')

        for person in data:
            name, phone, plicense, img_path, member_code, point, rent_time, return_time, cost, car_acident, is_renting, ID, PW = person
            member = Member(name, phone, plicense, img_path)
            member.member_code = member_code
            member.point = point
            member.is_renting = is_renting
            member.ID = ID
            member.PW = PW
            self.update_data(member, member.member_code)