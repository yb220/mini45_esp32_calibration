import unittest

from app.workflow import WORKFLOW_STAGES, WorkflowState


class WorkflowTests(unittest.TestCase):
    def test_stages_advance_in_fixed_order(self):
        state = WorkflowState()
        self.assertEqual(state.start(123), WORKFLOW_STAGES[0])
        seen = [state.stage]
        while state.active:
            state.advance()
            if state.stage:
                seen.append(state.stage)
        self.assertEqual(tuple(seen), WORKFLOW_STAGES)
        self.assertEqual(state.random_seed, 123)


if __name__ == "__main__":
    unittest.main()
