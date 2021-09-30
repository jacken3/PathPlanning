import configparser

class Config:
    def __init__(self,file):
        self.file=file
        self.con=configparser.ConfigParser()
        self.con.read(self.file,encoding='utf-8')
        self.Agent_config=dict(self.con.items("Agent"))
        self.Maze_config=dict(self.con.items("Maze"))
    

    







