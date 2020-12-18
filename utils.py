def show(*args):
    print("........", end="")
    for arg in args:
        print(arg, end="")


def show_if(boolean, if_true, if_false=" "):
    if boolean:
        show(if_true)
    else:
        show(if_false)
