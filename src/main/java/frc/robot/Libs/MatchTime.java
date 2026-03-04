package frc.robot.Libs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Dashboard;
import java.nio.ByteBuffer;
import java.util.function.Supplier;

/**
 * A serializable structure that maintains match period, remaining time, and game specific data.
 *
 * <p>DriverStation reports game specific message as a string. For the purposes of serialization
 * this instance must be of a fixed length, preventing strings of arbitrary length.
 *
 * <p>2026 REBUILT game sets the game specific message to 'B' or 'R' indicating the alliance whose
 * goal will go inactive first. Our assumption is that typical games would likely require game
 * specific states that can be packed into 64-bits. Therefore, default implementation reserves a
 * long to represent game-specific data. To allow correct interpretation of the game data and to
 * support future extensions a game season field is also present (FRC season year, e.g. 2026).
 */
public class MatchTime implements StructSerializable {
  public static enum Period {
    kNone((byte) 0),
    kAuto((byte) 1),
    kTeleop((byte) 2);

    Period(byte code) {
      this.code = code;
    }

    public final byte code;
  }

  public double remainingTime;
  public Period period;
  public final short gameSeason;
  public long gameData;

  public MatchTime(Period period, double remainingTime, int gameSeason, long gameData) {
    this.period = period;
    this.remainingTime = remainingTime;
    this.gameSeason = (short) gameSeason;
    this.gameData = gameData;
  }

  public MatchTime(int gameSeason) {
    this(Period.kNone, -1, gameSeason, 0);
  }

  /**
   * Updates this instance in place from the DriverStation's current data without setting the
   * game-specific data. You should typically invoke this method from your robot periodic code.
   */
  public void update() {
    this.period = getPeriod();
    this.remainingTime = DriverStation.getMatchTime();
  }

  /**
   * Updates this instance in place from the DriverStation's current data and sets the game-specific
   * data. You should typically invoke this method from your robot periodic code.
   */
  public void update(long gameData) {
    this.update();
    this.gameData = gameData;
  }

  /**
   * Constructs game-specific data for 2026 REBUILT game.
   *
   * <p>The encoded value identifies both robot's alliance and whether that alliance has its goal
   * being inactive first.
   *
   * <p>Bit 0 - set when robot's alliance is inactive first
   *
   * <p>Bit 1 - 0 for Blue alliance, 1 for Red alliance
   */
  public static final Supplier<Long> kGameData2026 =
      () -> {
        final var message = DriverStation.getGameSpecificMessage();
    final var bit1 = Dashboard.getAlliance() == DriverStation.Alliance.Red ? 1 : 0;

        var bit0 = 0;
        if (message.length() > 0) {
          switch (message.charAt(0)) {
            case 'B':
              bit0 = bit1 == 0 ? 1 : 0;
              break;
            case 'R':
              bit0 = bit1 == 1 ? 1 : 0;
              break;
              // ignore invalid data
          }
        }

        return ((long) bit1 << 1) | bit0;
      };

  private static Period getPeriod() {
    return DriverStation.isAutonomousEnabled()
        ? Period.kAuto
        : DriverStation.isTeleopEnabled() ? Period.kTeleop : Period.kNone;
  }

  public static final MatchTimeStruct struct = new MatchTimeStruct();

  public static final class MatchTimeStruct implements Struct<MatchTime> {
    @Override
    public Class<MatchTime> getTypeClass() {
      return MatchTime.class;
    }

    @Override
    public String getTypeName() {
      return "MatchTime";
    }

    @Override
    public int getSize() {
      return Struct.kSizeInt8 + Struct.kSizeDouble + Struct.kSizeInt16 + Struct.kSizeInt64;
    }

    @Override
    public String getSchema() {
      return "int8 period;double remainingTime;int16 gameSeason;int64 gameData;";
    }

    @Override
    public MatchTime unpack(ByteBuffer bb) {
      var period = Period.kNone;
      switch (bb.get()) {
        case 0:
          period = Period.kNone;
          break;
        case 1:
          period = Period.kAuto;
          break;
        case 2:
          period = Period.kTeleop;
          break;
        default:
          throw new Error("Unsupported match period");
      }

      return new MatchTime(period, bb.getDouble(), bb.getShort(), bb.getLong());
    }

    @Override
    public void pack(ByteBuffer bb, MatchTime value) {
      bb.put(value.period.code);
      bb.putDouble(value.remainingTime);
      bb.putShort(value.gameSeason);
      bb.putLong(value.gameData);
    }
  }
}