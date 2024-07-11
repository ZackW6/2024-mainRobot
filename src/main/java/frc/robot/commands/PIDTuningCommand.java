// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.WaitCommand;

// import java.util.ArrayList;
// import java.util.function.Consumer;
// import java.util.function.DoubleSupplier;

// import com.ctre.phoenix6.Utils;

// public class PIDTuningCommand extends Command {
//     private final DoubleSupplier outputVal;
//     private final Consumer<Double> P;
//     private double lowerLimit;
//     private double upperLimit;
//     private double mid = -1;
//     private final double tolerance = .1;
//     private final int minNumberOscilations = 20;

//     private boolean didOscilate = false;

//     public PIDTuningCommand(DoubleSupplier outputVal, Consumer<Double> P, double lowerLimit, double upperLimit, Subsystem requirement) {
//         this.outputVal = outputVal;
//         this.P = P;
//         this.lowerLimit = lowerLimit;
//         this.upperLimit = upperLimit;
//         this.addRequirements(requirement);
//     }

//     @Override
//     public void initialize(){
//         getDefineRangeCommand().schedule();
//     }

//     public Command getDefineRangeCommand(){
//         Command isOscilatingCommand = getIsOscilatingCommand();
//         return new FunctionalCommand(
//             //Init
//             ()->{
                
//             }, 
//             //Execute
//             ()->{
//                 if (!isOscilatingCommand.isScheduled()){
//                     if (mid!=-1){
//                         if(didOscilate){
//                             upperLimit = mid;
//                             didOscilate = false;
//                         }else{
//                             lowerLimit = mid;
//                         }
//                     }
//                     mid = (upperLimit+lowerLimit)/2;
//                     P.accept(mid);
//                     isOscilatingCommand.schedule();
//                 }
//             },
//             //On stop
//             (interrupted)->{
//                 double Pu = getPeriodCommand(lastVals);
//                 double Ku = (upperLimit+lowerLimit)/2;
//                 double Kp = 0.6 * Ku;//11870,7122
//                 double Ki = Pu / 2;//100
//                 double Kd = Kp * Pu / 8;
//                 System.out.println("P: "+Kp+ " I: "+Ki+" D: "+Kd);

//             },
//             //When to stop 5156.164 2578.082 2578.082
//              ()->(upperLimit-lowerLimit<tolerance)
//         );
//     }

//     private int i = 0;
//     private double lastTime = Utils.getCurrentTimeSeconds();
//     private ArrayList<Double> lastVals = new ArrayList<Double>();

//     private int oscilationCount = 0;
//     private double whole = 0;

//     public Command getIsOscilatingCommand(){
//         ArrayList<Double> vals = new ArrayList<Double>();
//         return new FunctionalCommand(
//             //Init
//             ()->{
//                 i = 0;
//                 vals.clear();
//                 lastTime = Utils.getCurrentTimeSeconds();
//                 oscilationCount = 0;
//                 whole = 0;
//             },
//             //Execute
//             ()->{
//                 if (Utils.getCurrentTimeSeconds()-lastTime>=.2){
//                     lastTime = Utils.getCurrentTimeSeconds();
//                     vals.add(outputVal.getAsDouble());
//                     whole += outputVal.getAsDouble();
//                     i++;
//                 }
//             },
//             //On stop
//             (interrupted)->{
//                 double mean = whole/vals.size();
//                 System.out.println("MEAN: "+mean+" SIZE: "+vals.size());
//                 int lastAboveBelow = 0;
//                 for (double val : vals){
//                     if (lastAboveBelow!=0){
//                         if (((val-mean>0) != (lastAboveBelow>0)) && (Math.abs(val-mean)>.1)){
//                             oscilationCount++;
//                         }
//                     }
//                     if (val-mean>0){
//                         lastAboveBelow = 1;
//                     }else{
//                         lastAboveBelow = -1;
//                     }
//                 }
//                 System.out.println("OSCILATION COUNT: "+oscilationCount);
//                 if (oscilationCount > minNumberOscilations) {
//                     didOscilate = true;
//                 }

//                 if (didOscilate){
//                     lastVals = vals;
//                 }
//             },
//             //When to stop
//              ()->(i>=100)
//         );
//     }

//     public double getPeriodCommand(ArrayList<Double> vals){
//         double whole = 0;
//         int oscilationCount = 0;
//         for (double val : vals){
//             whole+=val;
//         }
//         double mean = whole/vals.size();
//         int lastAboveBelow = 0;
//         for (double val : vals){
//             if (lastAboveBelow!=0){
//                 if (((val-mean>0) != (lastAboveBelow>0)) && (Math.abs(val-mean)>.1)){
//                     oscilationCount++;
//                 }
//             }
//             if (val-mean>0){
//                 lastAboveBelow = 1;
//             }else{
//                 lastAboveBelow = -1;
//             }
//         }

//         return vals.size()/(oscilationCount/2);
//     }

//     @Override
//     public void execute(){

//     }

//     @Override
//     public void end(boolean interrupted){

//     }

//     public boolean isFinished(){
//         return false;
//     }



// }