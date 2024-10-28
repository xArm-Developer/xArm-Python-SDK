To set up conda env:

In your terminal:
cd computer_vision/birds_eye_view/overhead2
conda create -n pcr_cv python=3.11.7
conda activate pcr_cv
pip install -r requirements_1.txt
pip install -r requirements_2.txt

If the last two commands error out, try this instead:
pip install -r requirements_no_version_1.txt
pip install -r requirements_no_version_2.txt

If this still errors out, and you are using an Apple device, dm Vishal on discord with your error.
If you are using a Windows, try installing cmake manually (https://cmake.org/download/), adding it to your path, verifying it is on your path by checking the cmake version, and then try re running the install commands.


To run birds_eye_view code:
cd computer_vision/birds_eye_view/overhead2
conda activate pcr_cv
python t2.py