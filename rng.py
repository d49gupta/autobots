adder = 12345
multiplier = 56789
dividor = 1928

def get_rand(key):
    key = (key*multiplier + adder) % dividor

    if abs(key) > 10000:
        key = int(key / 10000)
    
    if abs(key) % 2 == 0:
        key = -key

    return key

if __name__ == "__main__":
    ans = 16
    while (True):
        ans = get_rand(ans)
        if abs(ans) < 100:
            print("Condition has been met: ", ans)
            break
        
        print(ans)
