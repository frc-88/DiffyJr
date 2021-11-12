import edu.wpi.first.wpilibj.controller.StateSpaceControllerCoeffs;
import edu.wpi.first.wpilibj.controller.StateSpaceLoop;
import edu.wpi.first.wpilibj.controller.StateSpaceObserverCoeffs;
import edu.wpi.first.wpilibj.controller.StateSpacePlantCoeffs;
import edu.wpi.first.wpiutil.math.*;
import edu.wpi.first.wpiutil.math.numbers.*;

public class DiffSwerveCoeffs {
  public static StateSpacePlantCoeffs<N3, N2, N3> makeDiffSwervePlantCoeffs() {
    Matrix<N3, N3> A = MatrixUtils.mat(Nat.N3(), Nat.N3()).fill(1.0, 0.0026168234870946566, -0.0015137949316343605, 0.0, 0.08068567374139145, -0.4890050191403114, 0.0, 0.815008365233852, 0.7326923659284738);
    Matrix<N3, N2> B = MatrixUtils.mat(Nat.N3(), Nat.N2()).fill(0.01144831179965526, 0.0026811645217924736, 4.046960207604781, 0.5173203925108799, -0.8622006541848002, 3.3571996842569405);
    Matrix<N3, N3> C = MatrixUtils.mat(Nat.N3(), Nat.N3()).fill(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    Matrix<N3, N2> D = MatrixUtils.mat(Nat.N3(), Nat.N2()).fill(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    return new StateSpacePlantCoeffs<N3, N2, N3>(Nat.N3(), Nat.N2(), Nat.N3(), A, B, C, D);
  }

  public static StateSpaceControllerCoeffs<N3, N2, N3>
    makeDiffSwerveControllerCoeffs() {
    Matrix<N2, N3> K = MatrixUtils.mat(Nat.N2(), Nat.N3()).fill(0.3806632310629402, -0.009521442188937688, -0.14375183595989896, 0.10182353161669805, 0.2368117670074103, 0.17841645515011398);
    Matrix<N2, N3> Kff = MatrixUtils.mat(Nat.N2(), Nat.N3()).fill(0.0006817394366719805, 0.2392428215772076, -0.036866147342890614, 0.0002796934813536264, 0.06144236772447344, 0.28839924680322854);
    Matrix<N2, N1> Umin = MatrixUtils.mat(Nat.N2(), Nat.N1()).fill(-12.0, -12.0);
    Matrix<N2, N1> Umax = MatrixUtils.mat(Nat.N2(), Nat.N1()).fill(12.0, 12.0);
    return new StateSpaceControllerCoeffs<N3, N2, N3>(K, Kff, Umin, Umax);
  }

  public static StateSpaceObserverCoeffs<N3, N2, N3>
    makeDiffSwerveObserverCoeffs() {
    Matrix<N3, N3> K = MatrixUtils.mat(Nat.N3(), Nat.N3()).fill(0.9901952237543324, 3.728393979153534e-09, 4.009741015156001e-09, 3.728393979153651e-07, 0.9996001991812257, -4.672194162225529e-08, 4.0097410151537817e-07, -4.67219416222839e-08, 0.9996003517801721);
    return new StateSpaceObserverCoeffs<N3, N2, N3>(K);
  }

  public static StateSpaceLoop<N3, N2, N3> makeDiffSwerveLoop() {
    return new StateSpaceLoop<N3, N2, N3>(makeDiffSwervePlantCoeffs(),
                                          makeDiffSwerveControllerCoeffs(),
                                          makeDiffSwerveObserverCoeffs());
  }
}
