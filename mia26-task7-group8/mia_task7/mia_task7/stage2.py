import time
import asyncio

class stage2:
    def __init__(self, move_x: float, toggle_walls: bool):
        self.move_x = move_x
        self.toggle_walls = toggle_walls
        self.approach_distance = 0.55
        self.cross_distance = 0.70
        
async def run(self):
    success = await self.toggle_walls(False)
    
    if not success:
        print("Failed to toggle walls off")
        return False
    
    success = await self.move_x(self.approach_distance)
    
    if not success:
        print("Failed to move to approach distance")
        return False
    
    success = await self.move_x(self.cross_distance)
    
    if not success:
        print("Failed to move to cross distance")
        return False
    
    return True
    