import pygame
import math

# 初始化 Pygame
pygame.init()

# 设置窗口大小
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption('OBB Collision Detection')

# 定义颜色
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

# 定义 OBB 类
class OBB:
    def __init__(self, x, y, width, height, angle):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.angle = angle

    def get_corners(self):
        # 计算 OBB 的四个角点
        cos_a = math.cos(math.radians(self.angle))
        sin_a = math.sin(math.radians(self.angle))

        corners = [
            (self.x, self.y),
            (self.x + self.width * cos_a, self.y + self.width * sin_a),
            (self.x + self.width * cos_a - self.height * sin_a, self.y + self.width * sin_a + self.height * cos_a),
            (self.x - self.height * sin_a, self.y + self.height * cos_a)
        ]
        return corners

    def draw(self, screen, color):
        corners = self.get_corners()
        pygame.draw.polygon(screen, color, corners, 2)

def separating_axis_theorem(obb1, obb2):
    # 使用分离轴定理检测 OBB 碰撞
    def get_projection(corners, axis):
        min_proj = float('inf')
        max_proj = float('-inf')
        for corner in corners:
            proj = corner[0] * axis[0] + corner[1] * axis[1]
            min_proj = min(min_proj, proj)
            max_proj = max(max_proj, proj)
        return min_proj, max_proj

    def overlap(proj1, proj2):
        return proj1[0] <= proj2[1] and proj2[0] <= proj1[1]

    axes = []
    corners1 = obb1.get_corners()
    corners2 = obb2.get_corners()

    for i in range(4):
        edge = (corners1[i][0] - corners1[i - 1][0], corners1[i][1] - corners1[i - 1][1])
        axis = (-edge[1], edge[0])
        axes.append(axis)

    for i in range(4):
        edge = (corners2[i][0] - corners2[i - 1][0], corners2[i][1] - corners2[i - 1][1])
        axis = (-edge[1], edge[0])
        axes.append(axis)

    for axis in axes:
        proj1 = get_projection(corners1, axis)
        proj2 = get_projection(corners2, axis)
        if not overlap(proj1, proj2):
            return False

    return True

# 创建两个 OBB
obb1 = OBB(200, 50, 100, 50, 30) # 红色rect随鼠标移动
obb2 = OBB(0, 0, 100, 50, 0) # 绿色rect固定的实验对象

# 主循环
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEMOTION:
            obb1.x, obb1.y = event.pos
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 4:  # 滚轮向上
                obb1.angle += 5
            elif event.button == 5:  # 滚轮向下
                obb1.angle -= 5

    # 填充背景颜色
    screen.fill(WHITE)

    # 绘制 OBB
    obb1.draw(screen, RED)
    obb2.draw(screen, GREEN)

    # 检测碰撞
    if separating_axis_theorem(obb1, obb2):
        print("Collision Detected!")
    else:
        print("No Collision.")

    # 更新显示
    pygame.display.flip()

# 退出 Pygame
pygame.quit()
