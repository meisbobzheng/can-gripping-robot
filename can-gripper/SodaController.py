from enum import Enum

class SodaController:
    SPRITE = "sprite"
    COKE = "coca-cola"
    PEPSI = "pepsi"
    FANTA = "fanta"
    ROOT_BEER = "root beer"

    def get_soda_list(self) -> list[str] :
        return [self.SPRITE, self.COKE, self.PEPSI, self.FANTA, self.ROOT_BEER]
    
    def get_soda_command(self, soda: str) -> str:
        if soda == self.SPRITE:
            return "A green soda can with SPRITE as the label"
        elif soda == self.COKE:
            return "A red soda can with COKE as the label"
        elif soda == self.PEPSI:
            return "A blue soda can with PEPSI as the label"
        elif soda == self.FANTA:
            return "An orange soda can with FANTA as the label"
        elif soda == self.ROOT_BEER:
            return "A silver soda can with Barq's in cursive font as the label"
        else:
            raise Exception("Invalid soda")
