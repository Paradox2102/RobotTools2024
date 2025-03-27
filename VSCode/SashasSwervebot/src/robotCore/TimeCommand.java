package robotCore;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class TimeCommand extends ParallelDeadlineGroup {

    static class ComputeTime extends Command {
        private Timer m_timer = new Timer();

        @Override
        public void initialize() {
            Logger.log("ComputeTime", 2, "initialize()");
            m_timer.start();
            m_timer.reset();
        }

        @Override
        public void end(boolean interrupted) {
          Logger.log("ComputeTime", 2, String.format("Elapsed Time = %f", m_timer.get()));
        }
    }

    public TimeCommand(Command command) {
        super(command, new ComputeTime());
    }
}
