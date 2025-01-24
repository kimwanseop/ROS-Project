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
        self.is_renting = False

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

    def add_member(self, member:Member):
        while True :
            member_code = member.generate_code()
            if member_code not in self.member_dict:
                member.member_code = member_code
                self.member_dict[member_code] = member
                if member.name not in self.name:
                    self.name[member.name] = [member_code]
                else:
                    self.name[member.name].append(member_code)
                break
        print(member.member_code, self.member_dict)

    def del_member(self, member:Member):
        if member.member_code in self.member_dict:
            del(self.member_dict[member.member_code])
            if len(self.name[member.name])==0:
                del(self.name[member.name])
            else:
                self.name[member.name].remove(member.member_code)

