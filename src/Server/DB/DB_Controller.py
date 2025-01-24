import mysql.connector

class ASAP_DB():
    def __init__(self, db_name='asap', host='localhost', user='root', password='1234'):
        self.db_name = db_name
        self.host = host
        self.user = user
        self.password = password
        self.id_list = []
    
    def get_cursor(self):
        remote = mysql.connector.connect(
            host=self.host,
            user=self.user,
            password=self.password,
            database=self.db_name)
        cursor = remote.cursor()
        return cursor, remote
    
    def execute(self, query):
        cursor, remote = self.get_cursor()
        cursor.execute(query)
        remote.commit()
        cursor.close()
        remote.close()

    def info(self, query):
        cursor, remote = self.get_cursor()
        cursor.execute(query)
        self.show_execute(cursor)
        remote.commit()
        cursor.close()
        remote.close()


    def show_execute(self, cursor):
        result = cursor.fetchall()
        for data in result:
            print(data)

    def get_data(self, table_name):
        cursor, remote = self.get_cursor()
        query = f"SELECT * FROM {table_name}"
        cursor.execute(query)
        result = cursor.fetchall()
        remote.commit()
        cursor.close()
        remote.close()
        return result

    def insert_values(self, table_name, values):
        cursor, remote = self.get_cursor()
        query = f"INSERT INTO {table_name} VALUES ({values})"
        cursor.execute(query)
        self.info(f"SELECT * FROM {table_name}")
        remote.commit()
        cursor.close()
        remote.close()

    def select_values(self, table_name):
        cursor, remote = self.get_cursor()
        query = f"SELECT * FROM {table_name}"
        cursor.execute(query)
        self.show_execute(cursor)
        remote.commit()
        cursor.close()
        remote.close()
