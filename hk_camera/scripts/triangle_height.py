import math,sys

def triangle_height(alpha_deg, beta_deg, base_length):
    # 角度转弧度
    alpha = math.radians(alpha_deg)
    beta = math.radians(beta_deg)
    gamma = math.radians(180 - alpha_deg - beta_deg)
    print(f"alpha: {alpha}, beta: {beta}, gamma: {gamma}")
    # 计算高
    h = base_length * math.sin(alpha) * math.sin(beta) / math.sin(gamma)
    return h

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python triangle_height.py <alpha_deg> <beta_deg> <base_length>")
        sys.exit(1)
    
    alpha_deg = float(sys.argv[1])
    beta_deg = float(sys.argv[2])
    base_length = float(sys.argv[3])
    
    height = triangle_height(alpha_deg, beta_deg, base_length)
    print(f"The height from vertex A to base BC is: {height:.2f}")
# 示例
# print(triangle_height(30, 45, 10))


'''
A (顶点)
     /\
    /  \
   /    \
  /      \
 /        \
/__________\
B          C

已知：
- BC = b
- ∠ABC = α（底边左端的角）
- ∠BCA = β（底边右端的角）
要求：A到底边BC的距离h
'''