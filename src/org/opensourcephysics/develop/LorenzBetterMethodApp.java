/*
 * Lorenz Lyapunov "Better Method" - Simplified Implementation
 * Based on Wolf et al. algorithm and Gram-Schmidt orthonormalization
 */
package org.opensourcephysics.develop;
import org.opensourcephysics.controls.*;
import org.opensourcephysics.frames.*;
import org.opensourcephysics.display3d.simple3d.*;
import org.opensourcephysics.numerics.*;
import org.opensourcephysics.display.*;
import java.awt.*;
import java.text.DecimalFormat;

public class LorenzBetterMethodApp extends AbstractSimulation {
    DecimalFormat decimal5 = new DecimalFormat("#0.00000");
    
    Display3DFrame lorenzFrame = new Display3DFrame("Lorenz - Better Method");
    LorenzBetterMethod lorenz = new LorenzBetterMethod();
    
    PlotFrame lyapunovFrame = new PlotFrame("Time", "Lyapunov Exponent", "Lyapunov (Better Method)");
    Dataset lyapunovDataset = new Dataset(Color.RED);

    public LorenzBetterMethodApp() {
        lorenzFrame.setPreferredMinMax(-20.0, 20.0, -30.0, 30.0, 0.0, 50.0);
        lorenzFrame.setDecorationType(org.opensourcephysics.display3d.core.VisualizationHints.DECORATION_AXES);
        lorenzFrame.addElement(lorenz);
        
        lyapunovDataset.setConnected(true);
        lyapunovDataset.setLineColor(Color.RED);
        lyapunovDataset.setMarkerSize(-1);
        lyapunovDataset.setMaximumPoints(1000000);  // Prevent data loss
        lyapunovFrame.addDrawable(lyapunovDataset);
        lyapunovFrame.setAutoscaleX(true);
        lyapunovFrame.setAutoscaleY(true);
        
        lorenzFrame.setLocation(50, 100);
        lyapunovFrame.setLocation(500, 100);
        lyapunovFrame.setSize(400, 300);
    }

    public void initialize() {
        double x = control.getDouble("x");
        double y = control.getDouble("y");
        double z = control.getDouble("z");
        double perturbation = control.getDouble("perturbation");
        double sigma = control.getDouble("sigma");
        double rho = control.getDouble("rho");
        double beta = control.getDouble("beta");
        double dt = control.getDouble("dt");
        
        lorenz.initialize(x, y, z, perturbation, sigma, rho, beta);
        lorenz.ode_solver.initialize(dt);
        
        lyapunovDataset.clear();
        lyapunovFrame.repaint();
        
        control.println("=== BETTER METHOD (Simplified) ===");
        control.println("Expected LE ≈ 0.9056 for standard Lorenz");
        control.println("Your naive method at t=800: LE=0.86655");
        control.println("===================================");
        
        lorenzFrame.setVisible(true);
        lyapunovFrame.setVisible(true);
    }

    public void reset() {
        control.setValue("x", 1.0);
        control.setValue("y", 0.0);
        control.setValue("z", 0.0);
        control.setValue("perturbation", 1e-12);
        control.setValue("sigma", 10.0);
        control.setValue("rho", 28.0);
        control.setValue("beta", 8.0/3.0);
        control.setAdjustableValue("dt", 0.01);
        control.setAdjustableValue("print interval", 50.0);
        
        enableStepsPerDisplay(true);
        setStepsPerDisplay(10);
    }

    protected void doStep() {
        for (int i = 0; i < stepsPerDisplay; i++) {
            lorenz.doStep();
        }
        
        double time = lorenz.getTime();
        double lyapunov = lorenz.getCurrentLyapunov();
        double printInterval = control.getDouble("print interval");
        
        // Print at configurable intervals
        if (time % printInterval < 0.1) {
            control.print("t=" + String.format("%.0f", time) + " ");
            control.print("LE=" + decimal5.format(lyapunov) + " ");
            control.println();
        }
        
        if (time > 10.0) {
            lyapunovDataset.append(time, lyapunov);
        }
        
        lyapunovFrame.setMessage("t=" + String.format("%.1f", time) + ", λ=" + decimal5.format(lyapunov));
        lyapunovFrame.repaint();
    }
    
    public static void main(String[] args) {
        SimulationControl.createApp(new LorenzBetterMethodApp());
    }
}

/**
 * Simplified "Better Method" implementation
 * Uses one main trajectory + one perturbation vector evolved with Jacobian
 */
class LorenzBetterMethod extends Group implements ODE {
    // Extended state: [x, y, z, dx, dy, dz, t] where dx,dy,dz is perturbation
    double[] state = new double[7];
    
    double sigma = 10.0;
    double rho = 28.0;
    double beta = 8.0/3.0;
    
    // Lyapunov calculation
    double lyapunovSum = 0.0;
    
    // Visualization
    ODESolver ode_solver = new RK45MultiStep(this);
    Element ball = new ElementEllipsoid();
    ElementTrail trail = new ElementTrail();

    public LorenzBetterMethod() {
        ball.setSizeXYZ(1, 1, 1);
        ball.getStyle().setFillColor(Color.RED);
        trail.getStyle().setLineColor(Color.RED);
        trail.setMaximumPoints(3000);
        
        addElement(trail);
        addElement(ball);
        
        ode_solver.setStepSize(0.01);
    }

    public void initialize(double x, double y, double z, double perturbationSize, double sigma, double rho, double beta) {
        this.sigma = sigma;
        this.rho = rho;
        this.beta = beta;
        
        // Main trajectory
        state[0] = x;
        state[1] = y;
        state[2] = z;
        
        // Initial perturbation vector (configurable size, in x direction)
        state[3] = perturbationSize;
        state[4] = 0.0;
        state[5] = 0.0;
        state[6] = 0; // time
        
        lyapunovSum = 0.0;
        
        trail.clear();
        trail.addPoint(x, y, z);
        ball.setXYZ(x, y, z);
        
        System.out.printf("Better method initialized with perturbation = %.2e%n", perturbationSize);
    }

    protected void doStep() {
        // Calculate current perturbation magnitude before step
        double oldMag = Math.sqrt(state[3]*state[3] + state[4]*state[4] + state[5]*state[5]);
        
        // Step the system (main trajectory + perturbation)
        ode_solver.step();
        
        // Calculate new perturbation magnitude
        double newMag = Math.sqrt(state[3]*state[3] + state[4]*state[4] + state[5]*state[5]);
        
        // Update Lyapunov sum
        if (oldMag > 0 && newMag > 0 && state[6] > 0) {
            // Key insight: accumulate ln(growth) over time
            lyapunovSum += Math.log(newMag / oldMag);
            
            // Renormalize to prevent overflow (keeps perturbation small)
            // Use the same size as initial perturbation for consistency
            double targetSize = 1e-12; // Could make this configurable too
            double scale = targetSize / newMag;
            state[3] *= scale;
            state[4] *= scale;
            state[5] *= scale;
        }
        
        // Update visualization
        trail.addPoint(state[0], state[1], state[2]);
        ball.setXYZ(state[0], state[1], state[2]);
    }

    /**
     * ODE system: simultaneous evolution of trajectory and perturbation
     */
    public void getRate(double[] state, double[] rate) {
        double x = state[0];
        double y = state[1];
        double z = state[2];
        double dx = state[3];
        double dy = state[4];
        double dz = state[5];
        
        // Main Lorenz equations
        rate[0] = sigma * (y - x);
        rate[1] = x * (rho - z) - y;
        rate[2] = x * y - beta * z;
        
        // Linearized perturbation equations (Jacobian * perturbation)
        // J = [-σ    σ    0 ]
        //     [ρ-z  -1   -x ]
        //     [y     x   -β ]
        rate[3] = -sigma * dx + sigma * dy;
        rate[4] = (rho - z) * dx - dy - x * dz;
        rate[5] = y * dx + x * dy - beta * dz;
        
        rate[6] = 1; // time
    }
    
    public double[] getState() {
        return state;
    }
    
    public double getTime() {
        return state[6];
    }
    
    public double getCurrentLyapunov() {
        if (state[6] > 0) {
            return lyapunovSum / state[6];
        }
        return 0.0;
    }
}