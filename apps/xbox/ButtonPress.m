function [press,buttonflag] = ButtonPress(button,buttonflag)

if buttonflag
    press = 0;
    buttonflag = button;
else
    press = button;
    buttonflag = button;
end
end