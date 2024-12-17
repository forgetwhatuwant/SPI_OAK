from datetime import datetime  # 用于获取当前时间
from PIL import Image, ImageDraw, ImageFont
from lib.waveshare_epd import epd2in13_V4


# 初始化e-Paper
epd = epd2in13_V4.EPD()
epd.init()
epd.Clear(0xFF)  # 清空屏幕

# 加载自定义字体并设置字体大小
font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
font_size = 24  # 设置为较大的字体大小
font = ImageFont.truetype(font_path, font_size)

def display_on_epaper_inverse(text, update_time=True):
    image = Image.new('1', (epd.height, epd.width), 0)  # 创建一个黑色背景的图像
    draw = ImageDraw.Draw(image)
    
    # 显示系统状态文本，使用大字体
    draw.text((10, 10), text, font=font, fill=255)  # 使用白色字体

    if update_time:
        # 显示时间在右下角
        current_time = datetime.now().strftime("%H:%M:%S")
        
        # 获取文本边界框的大小
        bbox = draw.textbbox((0, 0), current_time, font=font)
        time_width = bbox[2] - bbox[0]
        time_height = bbox[3] - bbox[1]

        draw.text((epd.height - time_width - 10, epd.width - time_height - 10), current_time, font=font, fill=255)  # 使用白色字体

    epd.displayPartial(epd.getbuffer(image))


def display_on_epaper(text, update_time=True):
    image = Image.new('1', (epd.height, epd.width), 255)  # 创建一个白色背景的图像
    draw = ImageDraw.Draw(image)
    
    # 显示系统状态文本，使用大字体
    draw.text((10, 10), text, font=font, fill=0)

    if update_time:
        # 显示时间在右下角
        current_time = datetime.now().strftime("%H:%M:%S")
        
        # 获取文本边界框的大小
        bbox = draw.textbbox((0, 0), current_time, font=font)
        time_width = bbox[2] - bbox[0]
        time_height = bbox[3] - bbox[1]

        draw.text((epd.height - time_width - 10, epd.width - time_height - 10), current_time, font=font, fill=0)

    epd.displayPartial(epd.getbuffer(image))

