import thorpy

def run():
    application = thorpy.Application((800, 600), "ThorPy Overview")

    start = thorpy.Clickable("Start")
    thorpy.makeup.add_basic_help(start,"Start Button:")
    stop = thorpy.Clickable("Stop")
    thorpy.makeup.add_basic_help(stop,"Stop button")

    quit = thorpy.make_button("Quit",func=thorpy.functions.quit_menu_func)

    title_element = thorpy.make_text("Overview example", 22, (0, 0 , 225))

    elements = [start, stop, quit]
    central_box = thorpy.Box(elements=elements)
    central_box.fit_children(margins=(30,30)) #we want big margins
    central_box.center(x_shift=-10, y_shift=-10) #center on screen
    central_box.add_lift() #add a lift (useless since box fits children)
    central_box.set_main_color((220,220,220,180)) #set box color and opacity
    tractor_img = "data/Tractor_img.jpg"
    background = thorpy.Background.make(image=tractor_img,
                                        elements=[title_element, central_box])
    thorpy.store(background)

    menu = thorpy.Menu(background)
    menu.play()

    application.quit()



if __name__ == "__main__":
    run()
