import os
import sys
import time
import faulthandler
import pytest
from PyQt5.QtWidgets import QApplication, QLineEdit, QTextEdit
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtTest import QTest

from discopygal.solvers_infra import Scene
from discopygal_tools.solver_viewer.solver_viewer_main import SolverViewerGUI

BASIC_SCENE = "examples/basic_examples/basic_scene.json"
IS_CI = os.environ.get('CI') == '1'
DEFAULT_SUCCESS_STRING = 'Successfully found a path'

if not IS_CI:
    faulthandler.disable()


class GUITester:
    def __init__(self, gui, app, solve_success_string):
        self.app = app
        self.gui = gui
        self.solve_success_string = solve_success_string

    def set_gui(self, gui):
        self.gui = gui

    def wait_for_end_playing(self):
        # wait until animation stops
        while self.gui.is_queue_playing():
            self.app.processEvents()
            time.sleep(0.2)

    def trigger_action(self, action_name, wait_time=1):
        print(f"Triggering {action_name}")
        self.gui.__getattribute__("action" + action_name).trigger()
        # process events for a bit
        deadline = time.time() + wait_time
        while time.time() < deadline:
            self.app.processEvents()

    def get_widget_text(self, widget_name):
        widget = self.gui.__getattribute__(widget_name)
        if isinstance(widget, QLineEdit):
            return widget.text()
        if isinstance(widget, QTextEdit):
            return widget.toPlainText()

    @staticmethod
    def press_enter_on_active_window():
        app = QApplication.instance()
        widget = app.activeModalWidget() or app.activeWindow()
        if widget:
            print(f"Pressing Enter on {widget}")
            QTest.keyClick(widget, QtCore.Qt.Key_Enter)
        else:
            print("No active window to send Enter")

    @staticmethod
    def close_active_window():
        QApplication.instance().exit()

    def basic_check_gui(self):
        assert self.gui.scene_path == self.get_widget_text("scenePathEdit")
        assert self.gui.solver.__class__.__name__ == self.get_widget_text("solverName")

        self.trigger_action("Solve", 2)
        if self.solve_success_string:
            assert self.solve_success_string in self.get_widget_text("textEdit")

        QtCore.QTimer.singleShot(500, self.press_enter_on_active_window)
        self.trigger_action("Verify", 1)
        self.trigger_action("ShowGraph", 1)
        self.trigger_action("ShowPaths", 1)
        self.trigger_action("Grid", 0.2)
        self.trigger_action("ShowBoundingBox", 0.2)
        self.trigger_action("Play", 2)
        self.wait_for_end_playing()
        self.trigger_action("Clear", 0.2)

        assert '' == self.get_widget_text("scenePathEdit")

        # when done, close the main window → this exits app.exec_()
        self.gui.mainWindow.close()


class TestsGUI:
    def solver_viewer_test_case(self, testing_function, **kwargs):
        print("\nStarting GUI test")
        app = QtWidgets.QApplication(sys.argv)

        # Close popup at start
        QtCore.QTimer.singleShot(1000, GUITester.press_enter_on_active_window)

        # Schedule the testing function in the main thread
        QtCore.QTimer.singleShot(2000, lambda: testing_function(gui_tester))

        # Set css style sheet
        stream = QtCore.QFile(":/style.qss")
        stream.open(QtCore.QIODevice.ReadOnly)
        app.setStyleSheet(QtCore.QTextStream(stream).readAll())

        gui = SolverViewerGUI(kwargs.get("scene"), kwargs.get("solver"), kwargs.get("solver_file"))
        gui_tester = GUITester(gui, app, solve_success_string=kwargs.get("solve_success_string", DEFAULT_SUCCESS_STRING))

        # Run the event loop → exits when gui.mainWindow.close() is called
        app.exec_()

    def solver_viewer_basic_test(self, **kwargs):
        self.solver_viewer_test_case(GUITester.basic_check_gui, **kwargs)

    @pytest.skip
    def test_solver_viewer_with_semi_exact(self):
        def show_arrangement_gui(gui_tester: GUITester):
            gui_tester.trigger_action("Solve", 2)
            if gui_tester.solve_success_string:
                assert gui_tester.solve_success_string in gui_tester.get_widget_text("textEdit")

            QtCore.QTimer.singleShot(500, gui_tester.press_enter_on_active_window)
            gui_tester.trigger_action("Verify", 1)
            time.sleep(1)
            QtCore.QTimer.singleShot(500, gui_tester.close_active_window)
            gui_tester.trigger_action("ShowArrangement", 1)

            # when done, close the main window → this exits app.exec_()
            gui_tester.gui.mainWindow.close()

        self.solver_viewer_test_case(show_arrangement_gui, scene="scenes/legacy/1_monster_square_tight.json", solver_file="semi_path_solver_v8_s1.py", solver="SemiPathSolver")

    def test_solver_viewer_sanity(self):
        self.solver_viewer_basic_test(scene="scenes/legacy/1_monster_square_tight.json", solver="PRM")


@pytest.skip
def test_semi_exact_solver():
    from semi_path_solver_v8_s1 import SemiPathSolver
    solver = SemiPathSolver.init_default_solver()
    assert solver.solve(Scene.from_file("scenes/legacy/1_monster_square_tight.json")) is not None


def test_sanity():
    from discopygal.solvers.prm import PRM
    solver = PRM.init_default_solver()
    assert solver.solve(Scene.from_file("scenes/legacy/1_monster_square_tight.json")) is not None
